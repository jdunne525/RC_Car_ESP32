/*
   31 mar 2015
   This sketch display UDP packets coming from an UDP client.
   On a Mac the NC command can be used to send UDP. (nc -u 192.168.1.101 2390).

   Configuration : Enter the ssid and password of your Wifi AP. Enter the port number your server is listening on.

*/

#include <WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoOTA.h>

bool DebugMode = false;

int status = WL_IDLE_STATUS;
const char* ssid = "***REMOVED***";  //  your network SSID (name)
const char* pass = "***REMOVED***";       // your network password

unsigned int localPort = 9876;      // local port to listen for UDP packets
unsigned int debugPort = 23000;

unsigned int PWMFrequency = 20;       //20Hz is what we need for the steering to function properly
//1KHz pwm squeals like crazy...
//5KHz:  squeal isn't horrible, but don't go below about 600 for the PWM value.. torque seems low
//300Hz: much better!  okay down to about 350
//200Hz: okay.. 300 seems better
//100Hz: no good.  very non-linear.. we're delivering too much during the on pulses the motor barely slows at lower duty cycles

byte packetBuffer[512]; //buffer to hold incoming and outgoing packets
char newbuffer[32];

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
//WiFiUDP debugUdp;

int OTAInProcess = false;

int MotorForwardPin = 33;
int MotorBackwardPin = 32;
int MotorLeftPin = 12;
int MotorRightPin = 14;

int MotorForwardChannel = 0;
int MotorBackwardChannel = 1;
int MotorLeftChannel = 2;
int MotorRightChannel = 3;


int MotorSleepPin = 25;
int TestPin = 2;

int LEDPin = 15;
bool LEDOn = true;

int MotorSpeed = 600;
int Direction = 50;

int TurnSpeed = 0;
int TurnMidPoint = 50;
int TurnMaxSpeed = 100;
int TurnDeadband = 10;
int HardTurnPWM = 300;
int SoftTurnPWM = 150;
int TurnPWM = 0;

int PWMTicksPerTurnSpeed = 6;   //(HardTurnPWM - SoftTurnPWM) / (TurnMidPoint - TurnDeadband)

int CommandTimeout = 60000;

bool FwdActive = false;
int FwdLastMillis = 0;
bool BackActive = false;
int BackLastMillis = 0;
bool LeftActive = false;
int LeftLastMillis = 0;
bool RightActive = false;
int RightLastMillis = 0;
bool LightDebounce = false;
int LightDebounceMillis = 0;

//Notes on PWM controlled Turning:
//20Hz at 150 through 250 / 1024 counts seem to be good turning thresholds


#define analogWrite(pin, pwm) digitalWrite(pin, (pwm > 0))

void setup()
{

  pinMode(MotorForwardPin, OUTPUT);
  pinMode(MotorBackwardPin, OUTPUT);
  pinMode(MotorLeftPin, OUTPUT);
  pinMode(MotorRightPin, OUTPUT);
  pinMode(MotorSleepPin, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(TestPin, OUTPUT);

  digitalWrite(MotorForwardPin, LOW);
  digitalWrite(MotorBackwardPin, LOW);
  digitalWrite(MotorLeftPin, LOW);
  digitalWrite(MotorRightPin, LOW);
  digitalWrite(MotorSleepPin, HIGH);

//  ledcSetup(MotorForwardChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//  ledcSetup(MotorBackwardChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//  ledcSetup(MotorLeftChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//  ledcSetup(MotorRightChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//
//  ledcAttachPin(MotorForwardPin, MotorForwardChannel);
//  ledcAttachPin(MotorBackwardPin, MotorBackwardChannel);
//  ledcAttachPin(MotorLeftPin, MotorLeftChannel);
//  ledcAttachPin(MotorRightPin, MotorRightChannel);

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
}

void loop()
{

  
  if (FwdActive && millis() - FwdLastMillis > CommandTimeout) {
    FwdActive = false;
    digitalWrite(MotorForwardPin, LOW);
    analogWrite(MotorForwardPin, 0);
  }

  if (BackActive && millis() - BackLastMillis > CommandTimeout) {
    BackActive = false;
    digitalWrite(MotorBackwardPin, LOW);
    analogWrite(MotorBackwardPin, 0);
  }

  if (LeftActive && millis() - LeftLastMillis > CommandTimeout) {
    LeftActive = false;
    digitalWrite(MotorLeftPin, LOW);
  }

  if (RightActive && millis() - RightLastMillis > CommandTimeout) {
    RightActive = false;
    digitalWrite(MotorRightPin, LOW);
  }

  if (LightDebounce && millis() - LightDebounceMillis > 250) {
    LightDebounce = false;
  }
  
  int noBytes = Udp.parsePacket();
  if ( noBytes ) {

    //debugUdp.beginPacket(debugUdp.remoteIP(), debugUdp.remotePort());

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


    //debugUdp.println();
    //debugUdp.endPacket();

//    Serial.println();
  } // end if

  ArduinoOTA.handle();
  while (OTAInProcess) {
    ArduinoOTA.handle();
  }
}

unsigned long StrToLong(byte *str, int StartIndex, int Len, int base){
  for (int i = 0; i < 9; i++) newbuffer[i] = 0;
  int j = 0;
  for (int i = StartIndex; i < StartIndex + Len; i++) {
    if (packetBuffer[i] < '0') break;
    newbuffer[j++] = packetBuffer[i];
  }
  return (strtoul(newbuffer, NULL, 16));
}

void  GoForward() {
  digitalWrite(MotorBackwardPin, LOW);
  analogWrite(MotorBackwardPin, 0);

  if (MotorSpeed >= 1020) {
    digitalWrite(MotorForwardPin, true);
  }
  else {
    analogWrite(MotorForwardPin, MotorSpeed);
  }
  
  FwdActive = true;
  FwdLastMillis = millis();
//  printf("Fwd");
}

void  Stop() {
  BackActive = false;
  digitalWrite(MotorBackwardPin, HIGH);
  analogWrite(MotorBackwardPin, 0);
  FwdActive = false;
  digitalWrite(MotorForwardPin, HIGH);
  analogWrite(MotorForwardPin, 0);
//  printf("Stop");
}

void  GoBackward() {
  digitalWrite(MotorForwardPin, LOW);
  analogWrite(MotorForwardPin, 0);

  if (MotorSpeed >= 1020) {
    digitalWrite(MotorBackwardPin, true);
  }
  else {
    analogWrite(MotorBackwardPin, MotorSpeed);
  }
  
  BackActive = true;
  BackLastMillis = millis();
//  printf("Back");
}

void  GoNeutral() {
  BackActive = false;
  digitalWrite(MotorBackwardPin, LOW);
  analogWrite(MotorBackwardPin, 0);
  FwdActive = false;
  digitalWrite(MotorForwardPin, LOW);
  analogWrite(MotorForwardPin, 0);
//  printf("Neutral");
}

void HandleTurnSpeed() {
  //Serial.printf("Turnspeed: %d  %d %d %d %d\n", TurnSpeed, newbuffer[0], newbuffer[1], newbuffer[2], newbuffer[3]);

  if (TurnSpeed <= TurnMidPoint - TurnDeadband) {
    TurnPWM = PWMTicksPerTurnSpeed * ((TurnMidPoint - TurnDeadband) - TurnSpeed) + SoftTurnPWM;
    //Serial.print("left: ");
    //Serial.printf("%d\n", TurnPWM);

    LeftActive = true;
    LeftLastMillis = millis();
    
    if (TurnPWM > HardTurnPWM) TurnPWM = HardTurnPWM;
    if (TurnPWM == HardTurnPWM)  {
      digitalWrite(MotorLeftPin, HIGH);
      digitalWrite(MotorRightPin, LOW);
    }
    else {
      analogWrite(MotorLeftPin, TurnPWM);
      analogWrite(MotorRightPin, 0);
    }
  }
  else if (TurnSpeed >= TurnMidPoint + TurnDeadband) {
    TurnPWM = PWMTicksPerTurnSpeed * (TurnSpeed - (TurnMidPoint + TurnDeadband)) + SoftTurnPWM;
    //Serial.print("right: ");
    //Serial.printf("%d\n", TurnPWM);

    RightActive = true;
    RightLastMillis = millis();
    
    if (TurnPWM > HardTurnPWM) TurnPWM = HardTurnPWM;
    if (TurnPWM == HardTurnPWM) {
      digitalWrite(MotorRightPin, HIGH);
      digitalWrite(MotorLeftPin, LOW);
    }
    else {
      analogWrite(MotorRightPin, TurnPWM);
      analogWrite(MotorLeftPin, 0);
    }
  }
  else {
    analogWrite(MotorLeftPin, 0);
    analogWrite(MotorRightPin, 0);
  }
}

boolean cmdStartsWith(byte *cmd, const char *st) { // checks if cmd starts with st
  for (int i = 0; ; i++) {

    //Serial.write(' ');
    //Serial.write(cmd[i]);
    //Serial.write(st[i]);

    if (st[i] < '0') {
      //Serial.print("ValidCmd\n");
      return true;
    }
    if (cmd[i] < '0') {
      //Serial.print("InvalidCmd1\n");
      return false;
    }
    if (cmd[i] != st[i]) {
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

void  OTASetup()
{
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}


////Untested as it turns out we don't need multiple frequencies for analogWrite()
////Soft PWM, which supports exactly one pin at a variable frequency and handled in the main loop()
//int SoftPWMPinNum = 0;
//int SoftPWMPeriod = 1000;
//int SoftPWMOffTime = 500;
//int SoftPWMOnTime = 500;
//
//void  SetSoftPWMPeriod(int PWMPeriod) {
//  SoftPWMPeriod = PWMPeriod;
//}
//void  SetSoftPWM(int PinToPWM, int PWMOnTime) {
//  SoftPWMPinNum = PinToPWM;
//  SoftPWMOnTime = PWMOnTime;
//  if (SoftPWMOnTime < SoftPWMPeriod) {
//    SoftPWMOffTime = SoftPWMPeriod - SoftPWMOnTime;
//  }
//  else {
//    SoftPWMOffTime = 0;
//  }
//}
//
//void  HandleSoftPWM() {
//  static unsigned long SoftPWMlastEventMicros = 0;
//
//  if (SoftPWMOffTime == 0) {
//    digitalWrite(SoftPWMPinNum, true);
//    return;
//  }
//  else if (SoftPWMOffTime == 0) {
//    digitalWrite(SoftPWMPinNum, false);
//    return;
//  }
//  if (digitalRead(SoftPWMPinNum)) {
//    if (micros() - SoftPWMlastEventMicros >= SoftPWMOnTime) {
//      digitalWrite(SoftPWMPinNum, false);
//      SoftPWMlastEventMicros = micros();
//    }
//  }
//  else {
//    if (micros() - SoftPWMlastEventMicros >= SoftPWMOffTime) {
//      digitalWrite(SoftPWMPinNum, true);
//      SoftPWMlastEventMicros = micros();
//    }
//  }
//}

