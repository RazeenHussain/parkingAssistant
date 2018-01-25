// INTERRUPT 2 ON PIN 21
// INTERRUPT 3 ON PIN 20

// PIN ASSIGNMENT

// MOTOR RIGHT PIN ASSIGNMENT
const int rMotorPWM=9;
const int rMotorLogicF=27;
const int rMotorLogicR=26;
const int rMotorEncoder=3;

// MOTOR LEFT PIN ASSIGNMENT
const int lMotorPWM=8;
const int lMotorLogicF=23;
const int lMotorLogicR=22;
const int lMotorEncoder=2;  

// SENSOR PIN ASSIGNMENT
const int sensorRightFront=A6;
const int sensorRightBack=A1;
const int sensorFrontLong=A2;
const int sensorFrontShort=A3;
const int sensorLeftFront=A4;
const int sensorLeftBack=A5;
const int sensorBackRight=A7;
const int sensorBackLeft=A0;

// VARIABLE DECLARATION

// RIGHT & LEFT MOTOR ENCODER TICKS COUNTER INITIALIZATION
unsigned long rMotorTicks=0;
unsigned long lMotorTicks=0;

// MOTOR PPR VALUE
const int rMotorPPR=180;
const int lMotorPPR=160;

// ENCODER TICKS VALUE
const long rEncoderTicks45=65;
const long rEncoderTicks90=130;
const long lEncoderTicks45=65;
const long lEncoderTicks90=130;

// MOTOR PWM ASSIGNMENT
volatile unsigned int motorPWM=120;
volatile unsigned int pwmPID=120;

// SENSOR LIMITS
const int limitShort=75;
const int limitLong=350;

// PID PARAMETERS
volatile double valueCurrent;
volatile double valueRequired;
const double gainP=1.6;
const double gainI=1;
const double gainD=.75;

// ERROR VARIABLE
volatile double errorP=0;
volatile double errorI=0;
volatile double errorD=0;

// PID TERM
volatile double outputPID;
volatile double valueP;
volatile double valueI;
volatile double valueD;

// MAXIMUM & MINIMUM VALUE ASSIGNMENT
const int valueMin=100;
const int valueMax=200;

// FUNCTION PROTOTYPE

// PID TERM FUNCTION PROTOTYPE
double termP(void);
double termI(void);
double termD(void);

// PID FUNCTION PROTOTYPE
void robotPID(void);

// MOTOR ANGULAR VELOCITY CALCULATION FUNCTION PROTOTYPE
double motorAngularVelocity(int, char);

// TURN FUNCTION PROTOTYPE
void turnRight45(void);
void turnLeft45(void);
void turnRight90(void);
void turnLeft90(void);

// LOGIC FUNCTION PROTOTYPE
void rMotorForward(void);
void rMotorReverse(void);
void rMotorBrake(void);
void lMotorForward(void);
void lMotorReverse(void);
void lMotorBrake(void);

// ROBOT MOVEMENT FUNCTION PROTOTYPE
void robotForward(void);
void robotReverse(void);
void robotBrake(void);
void robotTurnRight(void);
void robotTurnLeft(void);
void robotPWM(int);

// INTERRUPT FUNCTION PROTOTYPE
void rMotorInterrupt(void);
void lMotorInterrupt(void);

// ADC VALUE FUNCTION PROTOTYPE
int valueADC(int);

const unsigned char PS_32=(1<<ADPS2)|(ADPS0); 
const unsigned char PS_128=(1<<ADPS2)|(ADPS1); 

unsigned long time=0;

void setup (void)
{
  analogReference(DEFAULT);
  ADCSRA&=~PS_128;
  ADCSRA|=PS_32;
Serial.begin(9600);
  // MOTOR RIGHT PIN MODE
  pinMode(rMotorPWM, OUTPUT);
  pinMode(rMotorLogicF, OUTPUT);
  pinMode(rMotorLogicR, OUTPUT);
  pinMode(rMotorEncoder, INPUT);

  // MOTOR LEFT PIN MODE
  pinMode(lMotorPWM, OUTPUT);
  pinMode(lMotorLogicF, OUTPUT);
  pinMode(lMotorLogicR, OUTPUT);
  pinMode(lMotorEncoder, INPUT);

  // SENSOR PIN MODE
  pinMode(sensorRightFront, INPUT);
  pinMode(sensorRightBack, INPUT);
  pinMode(sensorLeftFront, INPUT);
  pinMode(sensorLeftBack, INPUT);
  pinMode(sensorFrontLong, INPUT);
  pinMode(sensorFrontShort, INPUT);
  pinMode(sensorBackRight, INPUT);
  pinMode(sensorBackLeft, INPUT);

  // INTERRUPT ASSIGNMENT
  attachInterrupt(rMotorEncoder,rMotorInterrupt,RISING);
  attachInterrupt(lMotorEncoder,lMotorInterrupt,RISING);
}

void loop (void)
{
/*
  robotForward();
  robotPWM(motorPWM);
  time=millis();
  while(1)
  {
    robotPID();
//    Serial.println(rMotorTicks);
//    Serial.println(lMotorTicks);
    
  }
  robotBrake();
  delay(500);
  
  while(1);
*/  

  robotForward();
  robotPWM(motorPWM);
  while (valueADC(sensorRightFront)>=limitShort || valueADC(sensorRightBack)>=limitShort)
  {
    Serial.println(rMotorTicks);
    robotPID();
  }
  
  robotBrake();
  delay(500);
  
  robotForward();
  robotPWM(motorPWM/2);
  while (valueADC(sensorRightBack)<=limitShort && valueADC(sensorRightFront)<=limitShort)
  {
    Serial.println(rMotorTicks);
    robotPID();
  }
 
  robotBrake();
  delay(500);

  robotReverse();
  robotPWM(motorPWM/2);
  rMotorTicks=0;
  lMotorTicks=0;
  while (valueADC(sensorRightBack)>=limitShort)
  {
    Serial.println(rMotorTicks);
    robotPID();
  }


  robotBrake();
  delay(500);


/*
  robotForward();
  robotPWM(motorPWM/2);
  delay(200);
  robotBrake();
  delay(500);
*/



  turnLeft45(); 
  robotBrake();
  delay(500);

/*  robotReverse();
  robotPWM(motorPWM/2);
  delay(1000);
*/
    

  robotReverse();
  robotPWM(motorPWM/2);
  rMotorTicks=0;
  lMotorTicks=0;
  while (rMotorTicks<=390 && lMotorTicks<=390)
  {
Serial.println(rMotorTicks);
     robotPID();
     if (rMotorTicks>=390 || lMotorTicks>=390)
     {
       robotBrake();
       break;
     }
  }
  robotBrake();
  delay(500);

  robotReverse();
  robotPWM(motorPWM/4);
  for(;(valueADC(sensorRightBack)<=180 && valueADC(sensorBackRight)<=180);)
  {
    Serial.println(rMotorTicks);
    if (valueADC(sensorRightBack)>=180 || valueADC(sensorBackRight)>=180)
    {
      break;
    }
 
  }


  robotBrake();
  delay(1000);

  turnRight45(); 
  robotBrake();
  delay(500);




/*
  while (valueADC(sensorBackRight)<=125 && valueADC(sensorBackLeft)<=125)
  {
    delay(75);
    robotPID();
  }

  robotBrake();
  delay(500);
*/
    
  robotForward();
  robotPWM(motorPWM/2);
  while (valueADC(sensorBackRight)>=80)
  {
    robotPID();
    Serial.println(rMotorTicks);
  }
  robotBrake();
  delay(500);

  while(1);

}

// PID FUNCTION
void robotPID(void)
{
  double percentChange=0;
  const long sampleTime=100;
  static long startTime=millis();
  long currentTime=millis();
  long changeTime=currentTime-startTime;
//  if (changeTime>=sampleTime)
//  { 
    valueRequired=motorAngularVelocity(lMotorPPR, 'l');
    valueCurrent=motorAngularVelocity(rMotorPPR, 'r');
    errorP=(valueRequired-valueCurrent)*100/valueRequired;
    valueP=termP();
  if ((millis()-startTime)<sampleTime)
  {
//    return;
  }
  else
  {  
 //   startTime=millis();
    valueI=termI()*((millis()-time)/1000);
    valueD=termD()/((millis()-time)/1000);
  }
  outputPID=1.3*(valueP+valueI+valueD+pwmPID);
/*    percentChange=outputPID/valueRequired;
    percentChange=(percentChange*pwmPID);
    pwmPID+=(int)percentChange;
*/
//pwmPID=(int)outputPID;
//    constrain(outputPID, valueMin, valueMax);
    analogWrite(rMotorPWM, constrain((int)outputPID, valueMin, valueMax));
    startTime=millis();
    Serial.print(pwmPID);
    Serial.print("   ");
    Serial.print(valueRequired);
    Serial.print("   ");
    Serial.print(valueCurrent);
    Serial.print("   ");
    Serial.println(outputPID);
    return;
  
}

// MOTOR ANGULAR VELOCITY CALCULATION FUNCTION
double motorAngularVelocity(int motorPPR, char ticks)
{
  unsigned long ticksCurrent; 
  double angularVelocity;
  unsigned long timeStart=millis();
  unsigned long ticksStart;
  if (ticks=='r')
  {
    ticksStart=rMotorTicks;
  }
  else if (ticks=='l')
  {
    ticksStart=lMotorTicks;
  }
  while(1)
  {
    if((millis()-timeStart)>=10)
    {
      if (ticks=='r')
      {
        ticksCurrent=rMotorTicks;
      }
      else if (ticks=='l')
      {
        ticksCurrent=lMotorTicks;
      }
      
      angularVelocity=((ticksCurrent-ticksStart)*3.142)*100/motorPPR;
//      Serial.println(motorTicks-ticksStart);
//      Serial.print(angularVelocity);
//      Serial.print(' ');

      break;
    }
  }
  return angularVelocity;
}

// PROPORTIONAL TERM CALCULATION FUNCTION
double termP(void)
{
  return (errorP*gainP);
}

// INTEGRAL TERM CALCULATION FUNCTION
double termI(void)
{
  errorI+=errorP;
  return (errorI*gainI);
}

// DERIVATIVE TERM CALCULATION FUNCTION
double termD(void)
{
  double value=(errorP-errorD)*gainD;
  errorD=errorP;
  return value;
}

// ADC VALUE CALCULATION FUNCTION
int valueADC(int sensor)
{
  unsigned int value=0;
  int reading;
  int i=0;
  
  for (i=1; i<=15; i++)
  {
    delay(15);
    reading=analogRead(sensor);
    if (reading>550 || reading<100)
    {
      continue;
    }
    else
    {
      value+=reading;
    }
  }
  value/=i;
  return value;
}

// RIGHT MOTOR ENCODER TICK COUNTING FUNCTION
void rMotorInterrupt(void)
{
  rMotorTicks++;
}

// LEFT MOTOR ENCODER TICK COUNTING FUNCTION
void lMotorInterrupt(void)
{
  lMotorTicks++;
}

// ROBOT FORWARD MOVEMENT LOGIC
void robotForward(void)
{
  rMotorForward();
  lMotorForward();
}

// ROBOT REVERSE MOVEMENT LOGIC
void robotReverse(void)
{
  rMotorReverse();
  lMotorReverse();
}

// ROBOT BRAKE MOVEMENT LOGIC
void robotBrake(void)
{
/*
  boolean x;
  boolean y;

  pinMode(lMotorLogicF, INPUT);
  pinMode(lMotorLogicR, INPUT);

  x=digitalRead(lMotorLogicF);
  y=digitalRead(lMotorLogicR);
  x=~x;
  y=~y;

  pinMode(lMotorLogicF, OUTPUT);
  pinMode(lMotorLogicR, OUTPUT);

  digitalWrite(lMotorLogicF, x);
  digitalWrite(lMotorLogicR, y);

  pinMode(rMotorLogicF, INPUT);
  pinMode(rMotorLogicR, INPUT);
  
  x=digitalRead(rMotorLogicF);
  y=digitalRead(rMotorLogicR);
  x=!x;
  y=!y;

  pinMode(rMotorLogicF, OUTPUT);
  pinMode(rMotorLogicR, OUTPUT);

  digitalWrite(rMotorLogicF, x);
  digitalWrite(rMotorLogicR, y);

  
  delay(100);
  */
  rMotorBrake();
  lMotorBrake();
}

// ROBOT TURN RIGHT MOVEMENT LOGIC
void robotTurnRight(void)
{
  rMotorReverse();
  lMotorForward();
}

// ROBOT TURN LEFT MOVEMENT LOGIC
void robotTurnLeft(void)
{
  rMotorForward();
  lMotorReverse();
}

// ROBOT PWM ASSLIGNMENT
void robotPWM(int pwm)
{
  errorP=0;
  errorI=0;
  errorD=0;
  analogWrite(rMotorPWM,pwm);
  analogWrite(lMotorPWM,pwm);
  pwmPID=pwm;
  time=millis();
/*  valueRequired=motorAngularVelocity(lMotorPPR, lMotorTicks);
  valueCurrent=motorAngularVelocity(rMotorPPR, rMotorTicks);
  errorP=valueRequired-valueCurrent;
  errorD=errorP;
  valueI=termI();
*/}

// RIGHT MOTOR FORWARD LOGIC
void rMotorForward(void)
{
  digitalWrite(rMotorLogicF, HIGH);
  digitalWrite(rMotorLogicR, LOW);
}

// RIGHT MOTOR REVERSE LOGIC
void rMotorReverse(void)
{
  digitalWrite(rMotorLogicF, LOW);
  digitalWrite(rMotorLogicR, HIGH);
}

// RIGHT MOTOR BRAKE LOGIC
void rMotorBrake(void)
{
  digitalWrite(rMotorLogicF, HIGH);
  digitalWrite(rMotorLogicR, HIGH);
}

// LEFT MOTOR FORWARD LOGIC
void lMotorForward(void)
{
  digitalWrite(lMotorLogicF, HIGH);
  digitalWrite(lMotorLogicR, LOW);
}

// LEFT MOTOR REVERSE LOGIC
void lMotorReverse(void)
{
  digitalWrite(lMotorLogicF, LOW);
  digitalWrite(lMotorLogicR, HIGH);
}

// LEFT MOTOR BRAKE LOGIC
void lMotorBrake(void)
{
  digitalWrite(lMotorLogicF, HIGH);
  digitalWrite(lMotorLogicR, HIGH);
}

// TURN RIGHT 45 DEGREES FUNCTION
void turnRight45(void)
{
  detachInterrupt(rMotorEncoder);
  detachInterrupt(lMotorEncoder);


  attachInterrupt(rMotorEncoder,rMotorInterrupt,RISING);
  attachInterrupt(lMotorEncoder,lMotorInterrupt,RISING);
  robotTurnRight();
  robotPWM(motorPWM);
  rMotorTicks=0;
  lMotorTicks=0;
  while (rMotorTicks<=rEncoderTicks45 && lMotorTicks<=lEncoderTicks45)
  {
    Serial.println(rMotorTicks);
//    robotPID();
    if (rMotorTicks>=rEncoderTicks45 || lMotorTicks>=lEncoderTicks45)
    {
      break;
    }
  }
}

// TURN LEFT 45 DEGREES FUNCTION
void turnLeft45(void)
{
  detachInterrupt(rMotorEncoder);
  detachInterrupt(lMotorEncoder);
  attachInterrupt(rMotorEncoder,rMotorInterrupt,RISING);
  attachInterrupt(lMotorEncoder,lMotorInterrupt,RISING);
  robotTurnLeft();
  robotPWM(motorPWM);
  rMotorTicks=0;
  lMotorTicks=0;
  while (rMotorTicks<=rEncoderTicks45 && lMotorTicks<=lEncoderTicks45)
  {
    Serial.println(rMotorTicks);
//    robotPID();
    if (rMotorTicks>=rEncoderTicks45 || lMotorTicks>=lEncoderTicks45)
    {
      break;
    }
  }
}

// TURN RIGHT 90 DEGREES FUNCTION
void turnRight90(void)
{
  detachInterrupt(rMotorEncoder);
  detachInterrupt(lMotorEncoder);
  attachInterrupt(rMotorEncoder,rMotorInterrupt,RISING);
  attachInterrupt(lMotorEncoder,lMotorInterrupt,RISING);
  robotTurnRight();
  robotPWM(motorPWM);
  rMotorTicks=0;
  lMotorTicks=0;
  while (rMotorTicks<=rEncoderTicks90 && lMotorTicks<=lEncoderTicks90)
  {
    robotPWM(motorPWM);
  }  
}

// TURN LEFT 90 DEGREES FUNCTION
void turnLeft90(void)
{
  detachInterrupt(rMotorEncoder);
  detachInterrupt(lMotorEncoder);


  attachInterrupt(rMotorEncoder,rMotorInterrupt,RISING);
  attachInterrupt(lMotorEncoder,lMotorInterrupt,RISING);
  robotTurnLeft();
  robotPWM(motorPWM);
  rMotorTicks=0;
  lMotorTicks=0;
  while (rMotorTicks<=rEncoderTicks90 && lMotorTicks<=lEncoderTicks90)
  {
    robotPID();
  }
}
