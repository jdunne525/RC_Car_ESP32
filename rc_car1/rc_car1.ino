
// 4-channel RC receiver for controlling
// an RC car / boat / plane / quadcopter / etc.
// using an ESP8266 and an Android phone with RoboRemo app

// Disclaimer: Don't use RoboRemo for life support systems
// or any other situations where system failure may affect
// user or environmental safety.

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Servo.h>

// config:

const char *ssid = "***REMOVED***";  // You will connect your phone to this Access Point
const char *pw = "***REMOVED***"; // and this is the password
IPAddress ip(192, 168, 0, 20); // From RoboRemo app, connect to this IP
IPAddress netmask(255, 255, 255, 0);
const int port = 9876; // and this port

const int chCount = 4; // 4 channels, you can add more if you have GPIOs :)
Servo servoCh[chCount]; // will generate 4 servo PWM signals
int chPin[] = {5, 0, 4, 13}; // ESP pins: GPIO 0, 2, 14, 12
int chVal[] = {1500, 1500, 1500, 1500}; // default value (middle)

int usMin = 700; // min pulse width
int usMax = 2300; // max pulse width


WiFiServer server(port);
WiFiClient client;


char cmd[100]; // stores the command chars received from RoboRemo
int cmdIndex;
unsigned long lastCmdTime = 60000;
unsigned long aliveSentTime = 0;


boolean cmdStartsWith(const char *st) { // checks if cmd starts with st
  for(int i=0; ; i++) {
    if(st[i]==0) return true;
    if(cmd[i]==0) return false;
    if(cmd[i]!=st[i]) return false;;
  }
  return false;
}


void exeCmd() { // executes the command from cmd

  lastCmdTime = millis();

  // example: set RoboRemo slider id to "ch0", set min 1000 and set max 2000
  
  if( cmdStartsWith("ch") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (int)atof(cmd+4);
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
  
  // invert channel:
  // example: set RoboRemo slider id to "ci0", set min -2000 and set max -1000
  
  if( cmdStartsWith("ci") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = -(int)atof(cmd+4);
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
  
  // use accelerometer:
  // example: set RoboRemo acc y id to "ca1"
  
  if( cmdStartsWith("ca") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (usMax+usMin)/2 + (int)( atof(cmd+4)*51 ); // 9.8*51 = 500 => 1000 .. 2000
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
  
  // invert accelerometer:
  // example: set RoboRemo acc y id to "cb1"
  
  if( cmdStartsWith("cb") ) {
    int ch = cmd[2] - '0';
    if(ch>=0 && ch<=9 && cmd[3]==' ') {
      chVal[ch] = (usMax+usMin)/2 - (int)( atof(cmd+4)*51 ); // 9.8*51 = 500 => 1000 .. 2000
      if(!servoCh[ch].attached()) {
        servoCh[ch].attach(chPin[ch], usMin, usMax);
      }   
      servoCh[ch].writeMicroseconds(chVal[ch]);
    }
  }
  
  
  
}



void setup() {

  delay(1000);

  /*for(int i=0; i<chCount; i++) {
    // attach channels to pins
    servoCh[i].attach(chPin[i], usMin, usMax);
    // initial value = middle
    chVal[i] = (usMin + usMax)/2;
    // update
    servoCh[i].writeMicroseconds( chVal[i] );
  }*/
  
  cmdIndex = 0;

  
  
  Serial.begin(115200);

  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP 
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP

  server.begin(); // start TCP server

  Serial.println("ESP8266 RC receiver 1.1 powered by RoboRemo");
  Serial.println((String)"SSID: " + ssid + "  PASS: " + pw);
  Serial.println((String)"RoboRemo app must connect to " + ip.toString() + ":" + port);
  
}


void loop() {

  // if contact lost for more than half second
  if(millis() - lastCmdTime > 500) {  
    for(int i=0; i<chCount; i++) {
      // set all values to middle
      servoCh[i].writeMicroseconds( (usMin + usMax)/2 );
      servoCh[i].detach(); // stop PWM signals
    }
  }

  

  if(!client.connected()) {
    client = server.available();
    return;
  }

  // here we have a connected client

  if(client.available()) {
    char c = (char)client.read(); // read char from client (RoboRemo app)

    if(c=='\n') { // if it is command ending
      cmd[cmdIndex] = 0;
      exeCmd();  // execute the command
      cmdIndex = 0; // reset the cmdIndex
    } else {      
      cmd[cmdIndex] = c; // add to the cmd buffer
      if(cmdIndex<99) cmdIndex++;
    }
  } 

  if(millis() - aliveSentTime > 500) { // every 500ms
    client.write("alive 1\n");
    // send the alibe signal, so the "connected" LED in RoboRemo will stay ON
    // (the LED must have the id set to "alive")
    
    aliveSentTime = millis();
    // if the connection is lost, the RoboRemo will not receive the alive signal anymore,
    // and the LED will turn off (because it has the "on timeout" set to 700 (ms) )
  }

}
