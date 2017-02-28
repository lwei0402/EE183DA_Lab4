# Lab 4: State Estimator  
### Team Member: Zhuoran Duan, Zixuan Fan, Likai Wei  

Lab4 is a state estimator implemented on a two wheeled robot to determine the current states of the robot. The state estimator of the lab is based on Extended Kalman Filter.    

  - **Bill of materials**  
  - **Hardware Design**  
  - **Modeling of robot movement**  
  - **Extended Kalman Filter**  
  - **Reference**  
  - **License**  

### Bill of materials
To use this API, the following hardware requirements have to be met:
* [ESP8266 micro-controller](https://en.wikipedia.org/wiki/ESP8266) - ESP8266 Micro-controller with Internet shield
* [ESP-12E motor shield](https://smartarduino.gitbooks.io/user-mannual-for-esp-12e-motor-shield/content/interface.html) - Motor shield for ESP8266 to drive the servos
* [Continuous servo x2](http://www.robotshop.com/en/9g-continuous-rotation-micro-servo.html?gclid=CjwKEAiAuc_FBRD7_JCM3NSY92wSJABbVoxBr_84gd3C95CZ1ej68wuzu-aVRZ8oqgd1agMPi5WTQxoCDYPw_wcB)
* [MPU 5060 Gyroscope sensor](http://playground.arduino.cc/Main/MPU-6050)
* [FC-51 Infrared sensor x2](http://www.playembedded.org/blog/en/2016/01/08/detecting-obstacle-with-ir-sensor-and-arduino/)
* Portable battery
* [Paper Robot](https://git.uclalemur.com/mehtank/paperbot)
* Mini USB cable

### Hardware Design
#### Main robot body
The two wheeled is based on the [paperbot project](https://git.uclalemur.com/mehtank/paperbot) The robot body is constructed by folding and cutting a printable paper chassis. Two continuous servos are connected to the MCU to maneuver the movement of the robot. By changing the rotation speeds of different servos, we can achieve movements in four directions, including: forward, backward, left and right. Controlling of the robot is implemented by a web server user interface allowing user to adjust the direction and speed of the movement. The whole system is powered by a portable battery placed inside the chassis through a micro-usb port.

#### Picture of assembled robot
![Alt text](img/car.png?raw=true "Optional Title")

#### Sensors
The FC-51 infrared sensor is used to determine the rotation speed of each wheels. Paper wheels has dark and bright color stripes on top of it. Infrared signal will be absorbed after hitting a dark surface. In contrast, bright color surface will reflect all infrared lights. The sensor is consisted of one infrared emitter, one infrared sensor and three pins for power and outputs. The emitter emits infrared light and the receiver can determine whether there are reflected lights to determine the existence of an obstacle. Three pins on the sensor represent Vcc, GND and outputs. Vcc and GND are used for powering the sensor itself, and the results of detection are transmitted to the MCU using the output pin. While the wheels are spinning, the receiver will see consecutively changing bright and dark colors, therefore we can calculate the speed of rotation by counting numbers of changes of dark and bright.

#### Sample code for obstacle detection
```
#define IR 2  
int detection = HIGH;    // no obstacle
void setup() {
  Serial.begin(9600);  
  pinMode(IR, INPUT);
}
void loop() {  
  detection = digitalRead(IR);
  if(detection == LOW){
    Serial.print("There is an obstacle!\n");
  }
  else{
    Serial.print("No obstacle!\n");
  }
  delay(500);    // in ms
}
```
#### MPU-6050 Gyroscope
MPU-6050 sensors contains an accelerometer and a gyro on a single chip. The gyroscope uses I2C communication protocol to talk between sensor and MCU. We can read raw values of all the parameters including acceleration and angular velocity in x, y and z directions. Since the chip only outputs raw values, scaling must be done in order to use the data.

#### Sample code for getting raw values
```
// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(333);
}
```

#### Modeling of robot movement
The movement of the robot can be modelled as a system with two controllable variables: velocities of left and right wheels.






### Implementation
Include the ESP8266 WiFi library and Arduino Servo library
```
#include <ESP8266WiFi.h>
#include <Servo.h>
```

Setup the web server confidentials and start the server.
```
WiFi.mode(WIFI_AP); //Our ESP8266-12E is an AccessPoint
WiFi.softAP("kfan_esp", "110"); // Provide the (SSID, password); .
server.begin(); // Start the HTTP Server

//Looking under the hood
Serial.begin(115200); //Start communication between the ESP8266-12E and the monitor window
IPAddress HTTPS_ServerIP= WiFi.softAPIP(); // Obtain the IP of the Server
Serial.print("Server IP is: "); // Print the IP to the monitor window
Serial.println(HTTPS_ServerIP);
```

Initialize the servo and calibrate to 0
```
myservo.attach(D2);  // attaches the servo on pin 2 to the servo object
myservo.write(0);
```

Checking connections.
```
WiFiClient client = server.available();
if (!client) {
  return;
}
//Looking under the hood
Serial.println("Somebody has connected :)");

//Read what the browser has sent into a String class and print the request to the monitor
String request = client.readStringUntil('\r');
//Looking under the hood
Serial.println(request);
```

Handle HTTP requests and spin the servo when user press the piano key down in the web interface.
```
if (request.indexOf("/ON") != -1){
  myservo.write(110);
  delay(500);
  myservo.write(0);
}
```

Inject html code and inline style onto the web interface.
```
String s = "HTTP/1.1 200 OK\r\n";
s += "Content-Type: text/html\r\n\r\n";
s += "<!DOCTYPE HTML>\r\n<html>\r\n";
s += "<button style=\"width: 100px; height: 500px; background: #fff; border: 3px solid #000; border-radius: 3px;\"><input value=\"Re\" type=\"button\" onclick=\"location.href='/ON'\"></button>";
s += "</html>\n";
```

Handle HTTP requests and spin the servo when user press the piano key down in the web interface.
```
if (request.indexOf("/ON") != -1){
  myservo.write(110);
  delay(500);
  myservo.write(0);
}
```

Flush the stream and reset the client info
```
client.flush(); //clear previous info in the stream
client.print(s); // Send the response to the client
delay(1);
Serial.println("Client disonnected"); //Looking under the hood
```

### TODO
 - Implement Arduino SD library to read html, css and javascript files into the board.  
 - 3D printing wrist and hand for the Robot Pianista.  
 - Multiple servos to simulate human hand.    

### Reference  
 * [Online virtual piano](http://piano-player.info)  
 * [ESP8266 motor shield diagram](http://amazingrobots.net/resources/motor_shield_diagram/)  
 * [ESP8266 board pin mappings](http://amazingrobots.net/resources/nodemcu_pinout/)  
 * [ESP8266-12E quick guide](http://ucla.mehtank.com/teaching/2016-17--02--ee183da/esp8266-12e-quick.pdf)  
 * [ESP8266 programming tutorial](http://www.instructables.com/id/Programming-the-ESP8266-12E-using-Arduino-software/)  

### License  
**MIT**

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [@thomasfuchs]: <http://twitter.com/thomasfuchs>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [keymaster.js]: <https://github.com/madrobby/keymaster>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]:  <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
