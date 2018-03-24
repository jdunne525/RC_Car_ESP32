/*
   Robo Remo powered RC car
   Written for ESP32

   Configuration : Enter the ssid and password of your Wifi AP. Enter the port number your server is listening on.

*/

#include "my_wifi_ssid_pass.h"  //file that contains MY_WIFI_SSID AND MY_WIFI_PASS only.  Comment this line and fill in the values directly.
#include <WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoOTA.h>
#include "freertos/timers.h"

bool DebugMode = false;

int status = WL_IDLE_STATUS;
const char* ssid = MY_WIFI_SSID;       // your network SSID (name)
const char* pass = MY_WIFI_PASS;       // your network password

unsigned int localPort = 9876;      // local port to listen for UDP packets
unsigned int debugPort = 23000;

byte packetBuffer[512]; //buffer to hold incoming and outgoing packets
char newbuffer[32];

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
//WiFiUDP debugUdp;

int OTAInProcess = false;

int LEDPin = 15;
bool LEDOn = true;

int CommandTimeout = 60000;

bool LightDebounce = false;
int LightDebounceMillis = 0;

int TurnSpeed = 0;
int MotorSpeed = 600;

void setup()
{

  SetupMotorIO();
  
  pinMode(LEDPin, OUTPUT);

  if (LEDOn) {
    digitalWrite(LEDPin, false);
  }
  //digitalWrite(LEDPin, HIGH);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // setting up Station AP
  WiFi.begin(ssid, pass);

  Serial.println("ES32 RC receiver 3.0 powered by RoboRemo");

  // Wait for connect to AP
  Serial.print("[Connecting]");
  Serial.print(ssid);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tries++;
    if (tries > 30) {
      break;
    }
  }
  Serial.println();

  printWifiStatus();

  Serial.println("Connected to wifi");
  Serial.print("Udp server started at port ");
  Serial.println(localPort);
  Udp.begin(localPort);
  //debugUdp.begin(debugPort);

  OTASetup();

  //Set up FreeRTOS timer 
  TimerHandle_t tmr;
  int id=1;
  int interval = 100;     //in mS
  tmr = xTimerCreate("MyTimer", interval, pdTRUE, ( void * )id, vTimerCallback);
  if( xTimerStart( tmr, 0 ) != pdPASS ) {
   printf("Timer start error");
  }
  
}

//Every 100mS routine:
void vTimerCallback(TimerHandle_t id) {

}

void loop()
{
  if (LightDebounce && millis() - LightDebounceMillis > 250) {
    LightDebounce = false;
  }
  
  int noBytes = Udp.parsePacket();
  if ( noBytes ) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, noBytes); // read the packet into the buffer

 DebugMode = true;
    if (DebugMode) {
      // display the packet contents in HEX
      for (int i = 1; i <= noBytes; i++) {
        //Serial.print(packetBuffer[i-1],HEX);
        Serial.write(packetBuffer[i - 1]);
        //debugUdp.write(packetBuffer[i-1]);
        if (i % 32 == 0) {
  //        Serial.println();
        }
      }
      Serial.println();
    }

    if (cmdStartsWith(packetBuffer, "Reset")) {
        pinMode(0, OUTPUT);
        digitalWrite(0, LOW);     //set pin 0 low prior to resetting to ensure the reset is done properly
        ESP.restart();
    }
    else if (cmdStartsWith(packetBuffer, "D") || cmdStartsWith(packetBuffer, "E")) {

      if (packetBuffer[1] == '1' && packetBuffer[2] == '1') {
        Stop();
      }
      else if (packetBuffer[1] == '1') {    //forward button
        GoForward();
      }
      else if (packetBuffer[2] == '1') {    //backward button
        GoBackward();
      }
      else {
        GoNeutral();
      }

      if (packetBuffer[3] == '1') {         //light button
        if (!LightDebounce) {
          LightDebounceMillis = millis();
          LightDebounce = true;
          if (LEDOn) {
            digitalWrite(LEDPin, false);
          }
          else {
            digitalWrite(LEDPin, true);
          }
          LEDOn = !LEDOn;
        }
      }

      if (cmdStartsWith(packetBuffer, "D")) {

        //Accelerometer interface:
        TurnSpeed = (int)StrToLong(packetBuffer, 5, 2, 16);    //*str, startindex, len, base
        HandleTurnSpeed();
        MotorSpeed = (int)StrToLong(packetBuffer, 8, 4, 16);    //*str, startindex, len, base
      }
      else {
        //Button interface:
        if (packetBuffer[4] == '1') TurnSpeed = 0;
        else if (packetBuffer[5] == '1') TurnSpeed = 100;
        else TurnSpeed = 50;
        HandleTurnSpeed();
        
        MotorSpeed = (int)StrToLong(packetBuffer, 7, 4, 16);    //*str, startindex, len, base
      }
      //printf("MotorSpeed: %d\n",MotorSpeed);
      //printf("TurnSpeed: %d\n",TurnSpeed);
    }
  }

  ArduinoOTA.handle();
  while (OTAInProcess) {
    ArduinoOTA.handle();
  }
}

