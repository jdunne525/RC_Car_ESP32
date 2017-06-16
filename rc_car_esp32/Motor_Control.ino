
int MotorSleepPin = 25;

int MotorForwardPin = 33;
int MotorBackwardPin = 32;
int MotorLeftPin = 12;
int MotorRightPin = 14;

int MotorForwardChannel = 0;
int MotorBackwardChannel = 1;
int MotorLeftChannel = 2;
int MotorRightChannel = 3;

unsigned int MotorPWMFrequency = 50;
//1KHz pwm squeals like crazy...
//5KHz:  squeal isn't horrible, but don't go below about 600 for the PWM value.. torque seems low
//300Hz: much better!  okay down to about 350
//200Hz: okay.. 300 seems better
//100Hz: no good.  very non-linear.. we're delivering too much during the on pulses the motor barely slows at lower duty cycles

int Direction = 50;

//Notes on PWM controlled Turning:
//20Hz at 150 through 250 / 1024 counts seem to be good turning thresholds
unsigned int TurnPWMFrequency = 20;       //20Hz is what we need for the steering to function properly


int TurnMidPoint = 50;
int TurnMaxSpeed = 100;
int TurnDeadband = 10;
int HardTurnPWM = 300;
int SoftTurnPWM = 150;
int TurnPWM = 0;

int PWMTicksPerTurnSpeed = 6;   //(HardTurnPWM - SoftTurnPWM) / (TurnMidPoint - TurnDeadband)


//#define analogWrite(pin, pwm) digitalWrite(pin, (pwm > 0))

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1

  uint32_t duty = (8191 / valueMax) * (value < valueMax ? value : valueMax);    //min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}


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

  ledcSetup(MotorForwardChannel, MotorPWMFrequency, LEDC_TIMER_13_BIT);
  ledcSetup(MotorBackwardChannel, MotorPWMFrequency, LEDC_TIMER_13_BIT);
  ledcSetup(MotorLeftChannel, TurnPWMFrequency, LEDC_TIMER_13_BIT);
  ledcSetup(MotorRightChannel, TurnPWMFrequency, LEDC_TIMER_13_BIT);

  ledcAttachPin(MotorForwardPin, MotorForwardChannel);
  ledcAttachPin(MotorBackwardPin, MotorBackwardChannel);
  ledcAttachPin(MotorLeftPin, MotorLeftChannel);
  ledcAttachPin(MotorRightPin, MotorRightChannel);
}



void  GoForward() {
  StopBackward();

  if (MotorSpeed >= 1010) {
    ledcAnalogWrite(MotorForwardChannel, 1023, 1023);
  }
  else {
    ledcAnalogWrite(MotorForwardChannel, MotorSpeed, 1023);
  }

 //  printf("Fwd");
}

void  StopForward() {
  ledcAnalogWrite(MotorForwardChannel, 0, 1023);
}

void  Stop() {
  ledcAnalogWrite(MotorBackwardChannel, 1023, 1023);
  ledcAnalogWrite(MotorForwardChannel, 1023, 1023);

//  printf("Stop");
}

void  GoBackward() {
  StopForward();

  if (MotorSpeed >= 1010) {
    ledcAnalogWrite(MotorBackwardChannel, 1023, 1023);
  }
  else {
    ledcAnalogWrite(MotorBackwardChannel, MotorSpeed, 1023);
  }
  
//  printf("Back");
}

void  StopBackward() {
  ledcAnalogWrite(MotorBackwardChannel, 0, 1023);
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
      ledcAnalogWrite(MotorLeftChannel, 1023, 1023);
      ledcAnalogWrite(MotorRightChannel, 0, 1023);
    }
    else {
      ledcAnalogWrite(MotorLeftChannel, TurnPWM, 1023);
      ledcAnalogWrite(MotorRightChannel, 0, 1023);
    }
  }
  else if (TurnSpeed >= TurnMidPoint + TurnDeadband) {
    TurnPWM = PWMTicksPerTurnSpeed * (TurnSpeed - (TurnMidPoint + TurnDeadband)) + SoftTurnPWM;
    //Serial.print("right: ");
    //Serial.printf("%d\n", TurnPWM);

    if (TurnPWM > HardTurnPWM) TurnPWM = HardTurnPWM;
    if (TurnPWM == HardTurnPWM) {

      ledcAnalogWrite(MotorRightChannel, 1023, 1023);
      ledcAnalogWrite(MotorLeftChannel, 0, 1023);
    }
    else {
      ledcAnalogWrite(MotorRightChannel, TurnPWM, 1023);
      ledcAnalogWrite(MotorLeftChannel, 0, 1023);
    }
  }
  else {
    ledcAnalogWrite(MotorRightChannel, 0, 1023);
    ledcAnalogWrite(MotorLeftChannel, 0, 1023);

  }
}

