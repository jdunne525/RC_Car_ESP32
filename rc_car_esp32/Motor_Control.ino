
int MotorSleepPin = 25;

int MotorForwardPin = 33;
int MotorBackwardPin = 32;
int MotorLeftPin = 12;
int MotorRightPin = 14;

int MotorForwardChannel = 0;
int MotorBackwardChannel = 1;
int MotorLeftChannel = 2;
int MotorRightChannel = 3;

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13


void  SetupMotorIO() {
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

  ledcSetup(MotorForwardChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//  ledcSetup(MotorBackwardChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//  ledcSetup(MotorLeftChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//  ledcSetup(MotorRightChannel, PWMFrequency, LEDC_TIMER_13_BIT);
//
  ledcAttachPin(MotorForwardPin, MotorForwardChannel);
//  ledcAttachPin(MotorBackwardPin, MotorBackwardChannel);
//  ledcAttachPin(MotorLeftPin, MotorLeftChannel);
//  ledcAttachPin(MotorRightPin, MotorRightChannel);

}



void  GoForward() {
  StopBackward();

//  if (MotorSpeed >= 1020) {
//    digitalWrite(MotorForwardPin, true);
//  }
//  else {
//    analogWrite(MotorForwardPin, MotorSpeed);
//  }

  ledcAnalogWrite(MotorForwardChannel, MotorSpeed, 1023);  
 //  printf("Fwd");
}

void  StopForward() {
  digitalWrite(MotorForwardPin, LOW);
  analogWrite(MotorForwardPin, 0);
  ledcAnalogWrite(MotorForwardChannel, 0, 1023);
}

void  Stop() {
  digitalWrite(MotorBackwardPin, HIGH);
  analogWrite(MotorBackwardPin, 0);
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
  
//  printf("Back");
}

void  StopBackward() {
  digitalWrite(MotorBackwardPin, LOW);
  analogWrite(MotorBackwardPin, 0);
}

void  GoNeutral() {
  StopBackward();
  StopForward();
//  printf("Neutral");
}

void HandleTurnSpeed() {
  //Serial.printf("Turnspeed: %d  %d %d %d %d\n", TurnSpeed, newbuffer[0], newbuffer[1], newbuffer[2], newbuffer[3]);

  if (TurnSpeed <= TurnMidPoint - TurnDeadband) {
    TurnPWM = PWMTicksPerTurnSpeed * ((TurnMidPoint - TurnDeadband) - TurnSpeed) + SoftTurnPWM;
    //Serial.print("left: ");
    //Serial.printf("%d\n", TurnPWM);
    
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

