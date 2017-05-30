/*
 * 31 mar 2015
 * This sketch display UDP packets coming from an UDP client.
 * On a Mac the NC command can be used to send UDP. (nc -u 192.168.1.101 2390). 
 *
 * Configuration : Enter the ssid and password of your Wifi AP. Enter the port number your server is listening on.
 *
 */

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoOTA.h>

int status = WL_IDLE_STATUS;
const char* ssid = "***REMOVED***";  //  your network SSID (name)
const char* pass = "***REMOVED***";       // your network password

unsigned int localPort = 9876;      // local port to listen for UDP packets
unsigned int debugPort = 23;

byte packetBuffer[512]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
WiFiUDP debugUdp;


int MotorForwardPin = 0;
int MotorBackwardPin = 5;

int MotorLeftPin = 4;
int MotorRightPin = 13;

int MotorSleepPin = 12;




void setup()
{

  pinMode(MotorForwardPin, OUTPUT);
  pinMode(MotorBackwardPin, OUTPUT);
  pinMode(MotorLeftPin, OUTPUT);
  pinMode(MotorRightPin, OUTPUT);
  pinMode(MotorSleepPin, OUTPUT);

  digitalWrite(MotorForwardPin, LOW);
  digitalWrite(MotorBackwardPin, LOW);
  digitalWrite(MotorLeftPin, LOW);
  digitalWrite(MotorRightPin, LOW);
  
  digitalWrite(MotorSleepPin, HIGH);
   
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // setting up Station AP
  WiFi.begin(ssid, pass);

  Serial.println("ESP8266 RC receiver 1.1 powered by RoboRemo");
  
  // Wait for connect to AP
  Serial.print("[Connecting]");
  Serial.print(ssid);
  int tries=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tries++;
    if (tries > 30){
      break;
    }
  }
  Serial.println();

  printWifiStatus();

  Serial.println("Connected to wifi");
  Serial.print("Udp server started at port ");
  Serial.println(localPort);
  Udp.begin(localPort);
  debugUdp.begin(debugPort);
  ArduinoOTA.begin();

}

void loop()
{
  int noBytes = Udp.parsePacket();
  if ( noBytes ) {

    debugUdp.beginPacket(debugUdp.remoteIP(), debugUdp.remotePort());

    /*
    Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" received from ");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
    */
    // We've received a packet, read the data from it
    Udp.read(packetBuffer,noBytes); // read the packet into the buffer

    // display the packet contents in HEX
    for (int i=1;i<=noBytes;i++){
      //Serial.print(packetBuffer[i-1],HEX);
      Serial.write(packetBuffer[i-1]);
      debugUdp.write(packetBuffer[i-1]);

      
      if (i % 32 == 0){
        Serial.println();
      }
      //else Serial.print(' ');
    } // end for

    if (cmdStartsWith(packetBuffer, "FD")) {
      digitalWrite(MotorForwardPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else if (cmdStartsWith(packetBuffer, "FU")) {
      digitalWrite(MotorForwardPin, LOW);   // turn the LED on (HIGH is the voltage level)
    }
    else if (cmdStartsWith(packetBuffer, "BD")) {
      digitalWrite(MotorBackwardPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else if (cmdStartsWith(packetBuffer, "BU")) {
      digitalWrite(MotorBackwardPin, LOW);   // turn the LED on (HIGH is the voltage level)
    }
    else if (cmdStartsWith(packetBuffer, "LD")) {
      digitalWrite(MotorLeftPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else if (cmdStartsWith(packetBuffer, "Lu")) {
      digitalWrite(MotorLeftPin, LOW);   // turn the LED on (HIGH is the voltage level)
    }
    else if (cmdStartsWith(packetBuffer, "RD")) {
      digitalWrite(MotorRightPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else if (cmdStartsWith(packetBuffer, "Ru")) {
      digitalWrite(MotorRightPin, LOW);   // turn the LED on (HIGH is the voltage level)
    }


    debugUdp.println();
    debugUdp.endPacket();

    Serial.println();
  } // end if

  ArduinoOTA.handle();
}


boolean cmdStartsWith(byte *cmd, const char *st) { // checks if cmd starts with st
  for(int i=0; ; i++) {

    //Serial.write(' ');
    //Serial.write(cmd[i]);
    //Serial.write(st[i]);
    
    if(st[i]<'0') {
      //Serial.print("ValidCmd\n");
      return true;
    }
    if(cmd[i]<'0') {
      //Serial.print("InvalidCmd1\n");
      return false;
    }
    if(cmd[i]!=st[i]) {
      //Serial.print("InvalidCmd2\n");
      return false;
    }
  }
  return false;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}
