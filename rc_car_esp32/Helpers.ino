
unsigned long StrToLong(byte *str, int StartIndex, int Len, int base){
  for (int i = 0; i < 9; i++) newbuffer[i] = 0;
  int j = 0;
  for (int i = StartIndex; i < StartIndex + Len; i++) {
    if (packetBuffer[i] < '0') break;
    newbuffer[j++] = packetBuffer[i];
  }
  return (strtoul(newbuffer, NULL, 16));
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

