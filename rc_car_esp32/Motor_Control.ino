
void  GoForward() {
  StopBackward();

//  if (MotorSpeed >= 1020) {
//    digitalWrite(MotorForwardPin, true);
//  }
//  else {
//    analogWrite(MotorForwardPin, MotorSpeed);
//  }

  ledcAnalogWrite(MotorForwardChannel, MotorSpeed, 1023);  
  
  FwdActive = true;
  FwdLastMillis = millis();
//  printf("Fwd");
}

void  StopForward() {
  digitalWrite(MotorForwardPin, LOW);
  analogWrite(MotorForwardPin, 0);
  ledcAnalogWrite(MotorForwardChannel, 0, 1023);
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

  StopForward();
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

void  StopBackward() {
  digitalWrite(MotorBackwardPin, LOW);
  analogWrite(MotorBackwardPin, 0);
}

void  GoNeutral() {
  BackActive = false;
  StopBackward();
  FwdActive = false;
  StopForward();
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

