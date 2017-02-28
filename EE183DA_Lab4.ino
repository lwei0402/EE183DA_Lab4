
/*
  Wireless Servo Control, with ESP as Access Point
  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.
  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP
  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon
    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system
  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Paper chassis
*/
#define Nsta 6     // Six state values: x, y, angle theta, velocity x', y', and angular velocity omega
#define Mobs 3     // Three measurements: angular velocity of the left wheel omega_left, right wheel omega_right and gyro reading theta_gyro
#define MPUAddr 0x68


#include <Arduino.h>
#include <Hash.h>
#include <FS.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>
#include <Servo.h>
#include <TinyEKF.h>
#include "debug.h"
#include "file.h"
#include "server.h"




int16_t ax,ay,az,gx,gy,gz;
double L = 4.25; // half width of the car
double r = 3; // radius of the wheel
double theta = 0; // angle of the robot

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
const int IR1 = D3;
const int IR2 = D4;

int det1;
int det2;
int prev_state_1 = 0;
int prev_state_2 = 0;
int prev_time_1 = 0;
int prev_time_2 = 0;
int count_1 = 0;
int count_2 = 0;
double speed_1;
double speed_2;
int u1;
int u2;
double vmax = 22;

const float pi = 3.14;
//int prev_loop_time = 0;
int t = 1;

double omega_left;
double omega_right;
double theta_gyro;
double result[6];

class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant 0.0001
            this->setQ(0, 0, .0001);
            this->setQ(1, 1, .0001);
            this->setQ(2, 2, .0001);
            this->setQ(3, 3, .0001);
            this->setQ(4, 4, .0001);
            this->setQ(5, 5, .0001);
            
            // Same for measurement noise
            this->setR(0, 0, .0001);
            this->setR(1, 1, .0001);
            this->setR(2, 2, .0001);
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            // process model is f(x)
            fx[0] = this->x[0] + this->x[3]*t;
            fx[1] = this->x[1] + this->x[4]*t;
            fx[2] = this->x[2] + this->x[5]*t;
            fx[3] = u1*(-1/2)*cos(theta)*vmax/90 + u2*(1/2)*cos(theta)*vmax/90;
            fx[4] = u1*(-1/2)*sin(theta)*vmax/90 + u2*(1/2)*sin(theta)*vmax/90;
            fx[5] = u1*vmax/(180*L) - u2*vmax/(180*L);
            
            // the process model Jacobian
            F[0][0] = 1;
            F[0][3] = t;
            F[1][1] = 1;
            F[1][4] = t;
            F[2][2] = 1;
            F[2][5] = t;
            
            // Measurement function
            hx[0] = this->x[3]*cos(theta)/r + this->x[4]*sin(theta)/r + this->x[5]*(L/r);  // left wheel angular velocity from current state
            hx[1] = this->x[3]*(-cos(theta))/r + this->x[4]*(-sin(theta))/r + this->x[5]*(L/r); // right wheel angular velocity from current state
            hx[2] = this->x[2];  // car(gyro) angle of the current state
            
            // Jacobian of measurement function
            H[0][3] = cos(theta)/r; 
            H[0][4] = sin(theta)/r ;
            H[0][5] = L/r ;
            H[1][3] = -cos(theta)/r; 
            H[1][4] = -sin(theta)/r ;
            H[1][5] = L/r ;
            H[2][2] = this->x[2];

            theta = this->x[2];
        }
};

Fuser ekf;


Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;
// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";
// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";
char* mDNS_name = "paperbot";
String html;
String css;

void setSleep(bool enable)
{ 
  Wire.beginTransmission(MPUAddr);
  Serial.println("began trans");
  Wire.write(0x6B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPUAddr, 1, true);
  uint8_t power= Wire.read();
  Serial.println("changing sleep bit");
  power = (enable) ? (power | 0b01000000) : (power & 0b10111111);
  Serial.println("setting sleep bit");
  Wire.beginTransmission(MPUAddr);
  Wire.write(0x6B);
  Wire.write(power);
  Wire.endTransmission(true);
}

void getGyroData(int16_t* gx, int16_t* gy, int16_t* gz)
{
  Wire.beginTransmission(MPUAddr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPUAddr, 6, true);
  *gx = Wire.read() << 8 | Wire.read();
  *gy = Wire.read() << 8 | Wire.read();
  *gz = Wire.read() << 8 | Wire.read();
}

void setGyroPres(uint8_t val)
{
  val &= 0b11;
  val = val << 3;
  Wire.beginTransmission(MPUAddr);
  Wire.write(0x1B);
  Wire.write(val);
  Wire.endTransmission(true);
}



void setup() 
{
    setupPins();
    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());
    setSleep(false); 
    setGyroPres(0);  
    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;
    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);
    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);
    stop();
}
void loop() 
{
    kalman();
    wsLoop();
    httpLoop();
    
}

void kalman()
{   
    getGyroData(&gx, &gy, &gz);
    theta_gyro = 250*gz*0.0175/32768;
    measureSpeed();
    omega_left = speed_1;
    omega_right = speed_2;
    double z[3] = {omega_left, omega_right, theta_gyro};
    ekf.step(z);
    for (int i = 0; i < 6; i++)
    {
      result[i] = ekf.getX(i);
    }   

// Report measured and predicted values
//    Serial.print(z[0]);
//    Serial.print(" ");
//    Serial.print(z[1]);
//    Serial.print(" ");
//    Serial.print(z[2]);
//    Serial.print(" ");
//    Serial.print(ekf.getX(0));
//    Serial.print(" ");
//    Serial.println(ekf.getX(1));
//    Serial.print(" ");
//    Serial.print(ekf.getX(2));
//    Serial.print(" ");
//    Serial.println(ekf.getX(3));
//    Serial.print(" ");
//    Serial.print(ekf.getX(4));
//    Serial.print(" ");
//    Serial.println(ekf.getX(5));
}

void measureSpeed()
{
   det1 = digitalRead(IR1);
   det2 = digitalRead(IR2);
  if(det1 != prev_state_1)
  {
    count_1++;
    prev_state_1 = det1;
  }
  if(det2 != prev_state_2)
  {
    count_2++;
    prev_state_2 = det2;
  }
  
  if(count_1 == 10)
  {
    count_1 = 0;
    speed_1 = 1000*pi/(millis()-prev_time_1);
    prev_time_1 = millis();
    Serial.println(speed_1);
  }

  if(count_2 == 10)
  {
    count_2 = 0;
    speed_2 = 1000*pi/(millis()-prev_time_2);
    prev_time_2 = millis();
    Serial.println(speed_2);
  }
}

//
// Movement Functions //
//
void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
  u1 = left;
  u2 = right;
}
void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}
void forward() {
  DEBUG("forward");
  drive(0, 180);
}
void backward() {
  DEBUG("backward");
  drive(180, 0);
}
void left() {
  DEBUG("left");
  drive(180, 180);
}
void right() {
  DEBUG("right");
  drive(0, 0);
}
//
// Setup //
//
void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");
    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");
    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    Wire.begin(12,13);
    DEBUG("Setup motor pins");
}
void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {
    String output = "( ";
    for(int i=0; i<6;i++)
    {
      String temp = String(result[i],2);
      output += temp;
      output += ", ";
    }
    output += ")";   
    char tx[60];
    output.toCharArray(tx,60);
//    sprintf(tx, "(%3d, %3d, %3d, %3d, %3d, %3d)", result[0], result[1], result[2], result[3], result[4], result[5]);
    //sprintf(tx, "(%3d, %3d)", speed_1, speed_2);
    wsSend(id, tx);
    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);
            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);
            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);
        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);
            if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                }
                else if(payload[1] == 'F') 
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }
            break;
    }
}





