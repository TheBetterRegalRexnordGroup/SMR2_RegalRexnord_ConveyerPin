#include <HardwareSerial.h>

// variables 
bool reverse = false;
bool insertComplete = false;
int encoderPin1 = 16;
int encoderPin2 = 2;

// Piston arduino pins
int CutterPiston1 = 53;
int CutterPiston2 = A6;
int HolderPiston = A2;
int AllingmentPiston = A3;

//Motor arduino pins
// Twister
int Motor1 = 13;      ///pwm pin 
int MotorDir1 = 27;
int MotorDir2 = 26;
// Twister
int Motor2 = 12;      // PWM pin
int MotorDir3 = 28;
int MotorDir4 = 29;
// Pusher
int Motor3 = 11;    //pwm pin
int MotorDir5 = 30;
int MotorDir6 = 31;

int Motor4 = 10; // PWM pin
int MotorDir7 = 32;
int MotorDir8 = 33;

int StepperDir = 15;
int stepPin = 14;
int StepsToNextHole = 58;          //Number of steps so that the next hole is alligned with the inserter.
int StepsToNextHoleSlow = 1;

// Start Button
int startButton1 = 4;
int startButton2 = 5;

// JoyStick
#define joyX A4
#define joyY A5

///////////////////////////////////-SETUP-/////////////////////////////////////////////////
void setup() {

  //set communication speed
  Serial.begin(9600);
  
  //initiate motors                                             setting contected pins as output to controll the H-bridge motor controllers
  pinMode(Motor1,OUTPUT); pinMode(Motor2,OUTPUT); pinMode(Motor3,OUTPUT); pinMode(Motor4, OUTPUT);
  pinMode(MotorDir1,OUTPUT); pinMode(MotorDir2,OUTPUT); pinMode(MotorDir3,OUTPUT); pinMode(MotorDir4,OUTPUT); pinMode(MotorDir5,OUTPUT); pinMode(MotorDir6,OUTPUT); pinMode(MotorDir7,OUTPUT); pinMode(MotorDir8,OUTPUT);
  
  //Start with motors off
  digitalWrite(MotorDir1,LOW); digitalWrite(MotorDir2,LOW); digitalWrite(MotorDir3,LOW); digitalWrite(MotorDir4,LOW); digitalWrite(MotorDir5,LOW); digitalWrite(MotorDir6,LOW); digitalWrite(MotorDir7,LOW); digitalWrite(MotorDir8,LOW);
  analogWrite(Motor1,0);  analogWrite(Motor2, 0); analogWrite(Motor3, 0); analogWrite(Motor4, 0); 

  // Initiate Pneumatics
  pinMode(HolderPiston, OUTPUT); pinMode(AllingmentPiston, OUTPUT); pinMode(CutterPiston1, OUTPUT); pinMode(CutterPiston2, OUTPUT);
  digitalWrite(HolderPiston, LOW); digitalWrite(AllingmentPiston, LOW); digitalWrite(CutterPiston1, LOW); digitalWrite(CutterPiston2, LOW);

  // Initiate StepperMotor
  pinMode(stepPin, OUTPUT);  pinMode(StepperDir, OUTPUT);
  
  // Initiate Optical Encoder
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
  
  // Initiate Start Button
  pinMode(startButton1, OUTPUT);
  pinMode(startButton2, INPUT_PULLUP); 

  Serial.println("Setup Done"); 
  
}

/////////////////////////////////////-LOOP-//////////////////////////
void loop() {

  int buttonMemory = digitalRead(startButton2);
 
  if (buttonMemory == 0) {
    MainLoop();
  }

  if (buttonMemory == 1) {
    JoyStick();
  }
  
}

void JoyStick() {
  // Reading JoyStick Values
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);

  // JoyStick Use
  if ((xValue > 1000) && (xValue < 1050)) {
    activateStepperMotorSlowForward();
  }

  if ((xValue > -10) && (xValue < 10)) {
    activateStepperMotorSlowBackward();
  }

  if ((yValue > 1000) && (yValue < 1050)) {
    activateAlignmentPiston();
  }

  if ((yValue > -10) && (yValue < 10)) {
    deactivateAlignmentPiston();
  }
}

//////////////////////// Function MainLoop //////////////////////////////////

//// Motor DC /////
void activateMotorDC(int MotorSpeed, int twisterDirection, int twisterSpeed) {

  // Activate Motor 1 & 2 - Twist Motor (Clockwise)
  if (twisterDirection == 0) {

    digitalWrite(MotorDir1,LOW); 
    digitalWrite(MotorDir2,HIGH); 

    analogWrite(Motor1, twisterSpeed);

    digitalWrite(MotorDir3,LOW); 
    digitalWrite(MotorDir4,HIGH); 

    analogWrite(Motor2, twisterSpeed);
  }
  
  // Activate Motor 1 & 2 - Twist Motor (Counter-Clockwise)
  if (twisterDirection == 1) {
    
    digitalWrite(MotorDir1,HIGH); 
    digitalWrite(MotorDir2,LOW); 

    analogWrite(Motor1, twisterSpeed);

    digitalWrite(MotorDir3,HIGH); 
    digitalWrite(MotorDir4,LOW); 

    analogWrite(Motor2, twisterSpeed);
  }
  
  // Activate Motor 3 - Push Motor
  digitalWrite(MotorDir5,LOW); 
  digitalWrite(MotorDir6,HIGH); 

  analogWrite(Motor3, MotorSpeed);

  // Activate Motor 4 - Push Motor
  digitalWrite(MotorDir7,HIGH); 
  digitalWrite(MotorDir8,LOW); 

  analogWrite(Motor4, MotorSpeed);
  
}

void activateMotorDCreverse(int MotorSpeed, int twisterDirection, int twisterSpeed) {

  // Activate Motor 1 & 2 - Twist Motor (Clockwise)
  if (twisterDirection == 0) {

    digitalWrite(MotorDir1,LOW); 
    digitalWrite(MotorDir2,HIGH); 

    analogWrite(Motor1, twisterSpeed);

    digitalWrite(MotorDir3,LOW); 
    digitalWrite(MotorDir4,HIGH); 

    analogWrite(Motor2, twisterSpeed);
  }
  
  // Activate Motor 1 & 2 - Twist Motor (Counter-Clockwise)
  if (twisterDirection == 1) {
    
    digitalWrite(MotorDir1,HIGH); 
    digitalWrite(MotorDir2,LOW); 

    analogWrite(Motor1, twisterSpeed);

    digitalWrite(MotorDir3,HIGH); 
    digitalWrite(MotorDir4,LOW); 

    analogWrite(Motor2, twisterSpeed);
  }
  
  // Activate Motor 3 - Push Motor
  digitalWrite(MotorDir5,HIGH); 
  digitalWrite(MotorDir6,LOW); 

  analogWrite(Motor3, MotorSpeed);

  // Activate Motor 4 - Push Motor
  digitalWrite(MotorDir7,LOW); 
  digitalWrite(MotorDir8,HIGH); 

  analogWrite(Motor4, MotorSpeed);
  
}

void deactivateMotorDC() {

  int MotorSpeed = 0; // Change Motor Speed

  // Activate Motor 1 - Push Motor
  digitalWrite(MotorDir1,LOW); 
  digitalWrite(MotorDir2,HIGH); 

  analogWrite(Motor1, MotorSpeed);

  // Activate Motor 2 - Twist Motor
  digitalWrite(MotorDir3,LOW); 
  digitalWrite(MotorDir4,HIGH); 

  analogWrite(Motor2, MotorSpeed);

  // Activate Motor 3 - Push Motor
  digitalWrite(MotorDir5,LOW); 
  digitalWrite(MotorDir6,HIGH); 

  analogWrite(Motor3, MotorSpeed);
}

///// Alignment Piston /////
void activateAlignmentPiston() {
  digitalWrite(AllingmentPiston, HIGH);   // Activate piston relay.
}

void deactivateAlignmentPiston() {
  digitalWrite(AllingmentPiston, LOW);   // Deactivate piston relay.
}

//// Cutting Piston /////
void activateCuttingPiston() {
  digitalWrite(CutterPiston1, HIGH);
  digitalWrite(CutterPiston2, HIGH);
}

void deactivateCuttingPiston() {
  digitalWrite(CutterPiston1, LOW);
  digitalWrite(CutterPiston2, LOW);
}

//// Push Down Piston ////
void activatePushDownPiston() {
  digitalWrite(HolderPiston, HIGH);
}

void deactivatePushDownPiston() {
  digitalWrite(HolderPiston, LOW);
}

///// Steppen Motor /////
// Automatic
void activateStepperMotorForward() {

  digitalWrite(StepperDir, HIGH);

  for (int i = 0; i < StepsToNextHole / 2; i++) {                    //rotate sprokket till next hole is alligned
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20000);
  }

  delay(5000);

  for (int i = 0; i < StepsToNextHole / 2; i++) {                    //rotate sprokket till next hole is alligned
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20000);
  }
}

// Manual
void activateStepperMotorSlowForward() {
  digitalWrite(StepperDir, HIGH);
  for (int i = 0; i < StepsToNextHoleSlow; i++) {                    //rotate sprokket till next hole is alligned
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(5000);
  }
}

void activateStepperMotorSlowBackward() {
  digitalWrite(StepperDir, LOW);
  for (int i = 0; i < StepsToNextHoleSlow; i++) {                    //rotate sprokket till next hole is alligned
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(5000);
  }
}

///// MainLoop /////
void MainLoop() {

  unsigned long startTime = 0;
  unsigned long delayTime = 800;
  int pistonState = 0;
  int counterPushTwister = 0;

  // Reset
  deactivatePushDownPiston();
  deactivateAlignmentPiston();
  deactivateCuttingPiston();

  delay(500);
  activatePushDownPiston();
  delay(500);
  activateAlignmentPiston();
  delay(500);
  activateMotorDC(255, 0, 255); 

   while (insertComplete == false) {
    // Gets Insert Info From Slave
    reverse = digitalRead(encoderPin1);
    insertComplete = digitalRead(encoderPin2);

    if (reverse == true) {
      activateMotorDCreverse(60, 0, 255); 
    }

    if (millis() - startTime >= delayTime) {
      startTime = millis();
      if (pistonState == 0) {
        deactivatePushDownPiston();
        delayTime = 100;
        pistonState = 1;
      } else {
        activatePushDownPiston();
        delayTime = 800;
        pistonState = 0;
      }
    }
  }

  // Reset inserComplete Variable
  reverse = false;
  insertComplete = false;

  deactivateMotorDC();
  delay(500);
  activateCuttingPiston();
  delay(500);
  deactivateCuttingPiston();
  delay(500);
  deactivateAlignmentPiston();
  delay(500);
  deactivatePushDownPiston();
  delay(500);
  activateStepperMotorForward();
  delay(5000);

  Serial.println("Finished Pin");  

}