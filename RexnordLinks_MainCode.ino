#include <HardwareSerial.h>

// variables 
float DesiredLength = 10.0;         // Depth set by the user on the LCD-screen /// Important: Preset of 40 cm, zo this needs to changed by the user.
float CurrentLength = 0.0;              // Current length measured by the Optical Encoder in CM

// Optical Encoder
int OpticalEncoderInternalPullup1 = 2; // Internal Pullup
int OpticalEncoderInternalPullup2 = 3; // Internal Pullup
float lengthPerStep = 0.0145; // Length Per Rotation

// Piston arduino pins
int CutterPiston = A1;
int HolderPiston = A3;
int AllingmentPiston = A2;

//Motor arduino pins
int Motor1 = 13;      ///pwm pin 
int MotorDir1 = 27;
int MotorDir2 = 26;

int Motor2 = 12;      // PWM pin
int MotorDir3 = 28;
int MotorDir4 = 29;

int Motor3 = 11;    //pwm pin
int MotorDir5 = 30;
int MotorDir6 = 31;

int SlipperySlopePin = 46;

int StepperDir = 15;
int stepPin = 14;
int StepsToNextHole = 58;          //Number of steps so that the next hole is alligned with the inserter.
int StepsToNextHoleSlow = 20;

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
  Serial1.begin(9600);
  
  //initiate motors                                             setting contected pins as output to controll the H-bridge motor controllers
  pinMode(Motor1,OUTPUT); pinMode(Motor2,OUTPUT); pinMode(Motor3,OUTPUT);
  pinMode(MotorDir1,OUTPUT); pinMode(MotorDir2,OUTPUT); pinMode(MotorDir3,OUTPUT); pinMode(MotorDir4,OUTPUT); pinMode(MotorDir5,OUTPUT); pinMode(MotorDir6,OUTPUT);
  pinMode(SlipperySlopePin, OUTPUT);    //output to controll relays for shakermotors on shaketable
  
  //Start with motors off
  digitalWrite(MotorDir1,LOW); digitalWrite(MotorDir2,LOW); digitalWrite(MotorDir3,LOW); digitalWrite(MotorDir4,LOW); digitalWrite(MotorDir5,LOW); digitalWrite(MotorDir6,LOW);
  analogWrite(Motor1,0);  analogWrite(Motor2, 0); analogWrite(Motor3, 0); 

  // Initiate Pneumatics
  pinMode(HolderPiston, OUTPUT); pinMode(AllingmentPiston, OUTPUT); pinMode(CutterPiston, OUTPUT);
  digitalWrite(HolderPiston, LOW); digitalWrite(AllingmentPiston, LOW); digitalWrite(CutterPiston, LOW);

  // Initiate StepperMotor
  pinMode(stepPin, OUTPUT);  pinMode(StepperDir, OUTPUT);
  
  // Initiate Optical Encoder
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
  attachInterrupt(0, ai0, RISING); // A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(1, ai1, RISING); // B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  
  // Initiate Start Button
  pinMode(startButton1, OUTPUT);
  pinMode(startButton2, INPUT_PULLUP);  
  
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

void activateMotorDC_Fast() {

  int MotorSpeed = 255; // Change Motor Speed

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
  digitalWrite(CutterPiston, HIGH);
}

void deactivateCuttingPiston() {
  digitalWrite(CutterPiston, LOW);
}

//// Push Down Piston ////
void activatePushDownPiston() {
  digitalWrite(HolderPiston, HIGH);
}

void deactivatePushDownPiston() {
  digitalWrite(HolderPiston, LOW);
}

//// Slippery Slope /////
void activateSlipperySlope() {
  digitalWrite(SlipperySlopePin, HIGH);
}

void deactivateSlipperySlope() {
  digitalWrite(SlipperySlopePin, LOW);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    CurrentLength = CurrentLength + lengthPerStep;

  } else{
    CurrentLength = CurrentLength - lengthPerStep;
  }
}
   
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    CurrentLength = CurrentLength - lengthPerStep;
  } else{
    CurrentLength = CurrentLength + lengthPerStep;
  }
}

///// Steppen Motor /////
void activateStepperMotorForward() {
  digitalWrite(StepperDir, HIGH);
  for (int i = 0; i < StepsToNextHole; i++) {                    //rotate sprokket till next hole is alligned
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20000);
  }

  delay(1000);
}


void activateStepperMotorSlowForward() {
  digitalWrite(StepperDir, HIGH);
  for (int i = 0; i < StepsToNextHoleSlow; i++) {                    //rotate sprokket till next hole is alligned
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20000);
  }

  delay(1000);
}

void activateStepperMotorSlowBackward() {
  digitalWrite(StepperDir, LOW);
  for (int i = 0; i < StepsToNextHoleSlow; i++) {                    //rotate sprokket till next hole is alligned
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20000);
  }

  delay(1000);
}

void MainLoop() {

  // Reset
  deactivatePushDownPiston();
  deactivateAlignmentPiston();
  deactivateCuttingPiston();

  delay(1000);
  activateAlignmentPiston();
  delay(1000);
  activatePushDownPiston();
  delay(1000);
  activateMotorDC_Fast(); 
  
  while (CurrentLength <= DesiredLength) {
    Serial1.println(CurrentLength);
    // Wait until Optical Encoder reaches Desired Length
  } 
  
  delay(10000);

  deactivateMotorDC();
  delay(3000);

  activateCuttingPiston();
  delay(1000);

  // Tot hier gaat het goed
  deactivateCuttingPiston();
  delay(2000);
  deactivatePushDownPiston();
  delay(2000);
  deactivateAlignmentPiston();
  delay(2000);
  activateStepperMotorForward();
  delay(2000);

  CurrentLength = 0.0;

  Serial.println("Insert Finished");  

}


