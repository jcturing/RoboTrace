#include <DRV8835MotorShield.h>
#include <QTRSensors.h>
#include <NewPing.h>
#include <Melodies.h>
#include <Button.h>
#include <Led.h>
#include <SoftwareSerial.h>
#include <Circuit.h>

//PINES & CONSTANTES
const int RX = 0;
const int TX = 1;
const int encoder1_A = 2;
const int encoder2_A = 3;
const int encoder1_B = 4;
const int encoder2_B = 5;
const int mux_s0 = 6;
const int motorA_1 = 7;
const int motorA_2 = 8;
const int motorB_1 = 9;
const int motorB_2 = 10;
const int mux_s1 = 11;
const int mux_s2 = 12;
const int mux_s3 = 13;
const int mux_z = A0;
const int buzzer = A1;
const int right_button = A2;
const int right_led = A3;
const int left_led = A4;
const int left_button = A5;

const int LEFT_NUM_SENSORS = 2;
const int CENTER_NUM_SENSORS = 8;
const int RIGHT_NUM_SENSORS = 1;

const bool DEBUG = false;

const int MAX_SPEED = 400;
const int INCREMENT = 25;

const float STRAIGHT_BEGIN = 0.1;
const float STRAIGHT_END = 0.75;
const int BRAKE_COEF = 4;

const float KP = 0.075;
const float KD = 0.275;

//VARIABLES GLOBALES
volatile long leftCount = 0;
volatile long rightCount = 0;

int runSpeed = 200;
int DEV = runSpeed + 20;
int calSpeed = 50;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

unsigned int leftSensorValues[LEFT_NUM_SENSORS];
unsigned int centerSensorValues[CENTER_NUM_SENSORS];
unsigned int rightSensorValues[RIGHT_NUM_SENSORS];

unsigned int leftLinePosition = 0;
unsigned int centerLinePosition = 0;
unsigned int rightLinePosition = 0;

int P, D, last = 0;
int lineErrorSpeed, encoderErrorSpeed;
int calibrationCounter = 0;
int circuitCounter = 0, actualSpeed, actualDEV, goalSpeed;
long aux_leftCount = 0, aux_rightCount = 0;

//DECLARACIONES
DRV8835MotorShield motors; //M1 -> LEFT - M2 -> RIGHT

QTRSensorsAnalog leftLightSensor((unsigned char[]) {mux_z, mux_z}, 
  LEFT_NUM_SENSORS, mux_s0, mux_s1, mux_s2, mux_s3, 'L');
QTRSensorsAnalog centerLightSensor((unsigned char[]) {mux_z, mux_z, mux_z, mux_z, mux_z, mux_z, mux_z, mux_z}, 
  CENTER_NUM_SENSORS, mux_s0, mux_s1, mux_s2, mux_s3, 'C');
QTRSensorsAnalog rightLightSensor((unsigned char[]) {mux_z}, 
RIGHT_NUM_SENSORS, mux_s0, mux_s1, mux_s2, mux_s3, 'R');

Melodies music(buzzer);

Button rightButton(right_button);
Button leftButton(left_button);

Led rightLed(right_led);
Led leftLed(left_led);

SoftwareSerial out(RX, TX);

Circuit circuit;

void setup() {
  //Pins declaration
  pinMode(encoder1_A, INPUT);
  pinMode(encoder2_A, INPUT);
  pinMode(encoder1_B, INPUT);
  pinMode(encoder2_B, INPUT);
  pinMode(motorA_1, OUTPUT);
  pinMode(motorA_2, OUTPUT);
  pinMode(motorB_1, OUTPUT);
  pinMode(motorB_2, OUTPUT);
  pinMode(mux_s0, OUTPUT);
  pinMode(mux_s1, OUTPUT);
  pinMode(mux_s2, OUTPUT);
  pinMode(mux_s3, OUTPUT);
  pinMode(mux_z, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(right_button, INPUT);
  pinMode(right_led, OUTPUT);
  pinMode(left_led, OUTPUT);
  pinMode(left_button, INPUT);  
 
  //motors.flipM1(true);
  //motors.flipM2(true);

  attachInterrupt(0, leftEncoderEvent, CHANGE);
  attachInterrupt(1, rightEncoderEvent, CHANGE);
  
  if(DEBUG){
    out.begin(38400);
    out.println("READY");
  }
  while(!rightButton.wasPressed()){}
  music.playBeep();
  waitForUser();
  calibration();
}

void loop() {
  if(!rightButton.wasPressed()){}
  else{
    music.playBeep();
    waitForUser();
    searchRun();
    while(true){
      if(!rightButton.wasPressed()){
        chooseSpeed();
      }
      else{
        music.playBeep();
        waitForUser();
        performRun();
      }
    }
  }
}


//FUNCIONES

void calibration(){
  leftLed.turnOn();
  for (int i = 0; i < 100; i++){
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
    if(calibrationCounter == 0){
      leftMotorSpeed = calSpeed;
      if(leftCount >= 450){
        calibrationCounter++;
      }
    } 
    else if(calibrationCounter == 1){
      leftMotorSpeed = -calSpeed;
      if(leftCount <= 80){
        calibrationCounter++;
      }
    }
    else if(calibrationCounter == 2){
      rightMotorSpeed = calSpeed;
      if(rightCount >= 450){
        calibrationCounter++;
      }
    }
    else if(calibrationCounter == 3){
      rightMotorSpeed = -calSpeed;
      if(rightCount <= 75){
        calibrationCounter++;
      }
    }
    leftLightSensor.calibrate();
    centerLightSensor.calibrate();
    rightLightSensor.calibrate();
    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
  }
  motors.setSpeeds(0, 0);
  leftLed.turnOff();
}

void waitForUser(){ 
  motors.setSpeeds(0,0);
  delay(1000);
}

void chooseSpeed(){
  if(leftButton.wasPressed()){
    music.playBeep();
    if(runSpeed + INCREMENT < MAX_SPEED) runSpeed = runSpeed + INCREMENT;
    else runSpeed = MAX_SPEED;
    DEV = runSpeed + 20;
    if(DEBUG) out.println(runSpeed);
    delay(500);
  }
}

void searchRun(){
  bool done = false, ini = false;
  bool right_once = false, left_once = false, right_crossLine = false, left_crossLine = false;
  long aux_leftCount = 0, aux_rightCount = 0, aux_leftDif = 0, aux_rightDif = 0;
  leftLed.turnOn();
  last = 0;
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
  
  while(!done){
    leftLinePosition = leftLightSensor.readLine(leftSensorValues);
    centerLinePosition = centerLightSensor.readLine(centerSensorValues);
    rightLinePosition = rightLightSensor.readLine(rightSensorValues);
    
    //RIGHT SENSOR
    if(rightLinePosition == 7001 && !right_once){ 
      right_once = true;
      if(!right_crossLine && straightLine()){
        music.playBeep();
        if(!ini){
          aux_leftCount = leftCount;
          aux_rightCount = rightCount;
          ini = true;
        }
        else{
          aux_leftDif = leftCount - aux_leftCount;
          aux_rightDif = rightCount - aux_rightCount;
          circuit.add(aux_leftDif, aux_rightDif);
          done = true;
        }
      }
      else right_crossLine = false;
    }
    else if(rightLinePosition != 7001){
      right_once = false;
    }
    
    //LEFT SENSOR
    if(leftLinePosition == 7001 && !left_once){ 
      left_once = true;
      if(!left_crossLine){
        music.playBeep();
        aux_leftDif = leftCount - aux_leftCount;
        aux_rightDif = rightCount - aux_rightCount;
        circuit.add(aux_leftDif, aux_rightDif);
        aux_leftCount = leftCount;
        aux_rightCount = rightCount;
      }
      else left_crossLine = false;
    }
    else if(leftLinePosition != 7001){
      left_once = false;
    }

    //CENTER SENSOR
    if(centerLinePosition == 0){
      leftMotorSpeed = -DEV;
      rightMotorSpeed = DEV;
    }
    else if(centerLinePosition == 7000){
       leftMotorSpeed = DEV;
       rightMotorSpeed = -DEV;
    }
    else if(centerLinePosition == 7001){
      leftMotorSpeed = runSpeed;
      rightMotorSpeed = runSpeed;
      left_crossLine = true;
      right_crossLine = true;
    }
    else{
        P = centerLinePosition - 3500;
        D = P - last;
        lineErrorSpeed = (P * KP) + (D * KD);    
        last = P;
        
        leftMotorSpeed = runSpeed + lineErrorSpeed;
        rightMotorSpeed = runSpeed - lineErrorSpeed; 
        if(leftMotorSpeed > DEV) leftMotorSpeed = DEV;
        else if(leftMotorSpeed < -DEV) leftMotorSpeed = -DEV;
        if(rightMotorSpeed > DEV) rightMotorSpeed = DEV;
        else if(rightMotorSpeed < -DEV) rightMotorSpeed = -DEV;
    }      
    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
  }
  
  aux_rightCount = rightCount;
  last = 0;
  while(rightCount < aux_rightCount + 1500){
    centerLinePosition = centerLightSensor.readLine(centerSensorValues);
    P = centerLinePosition - 3500;
    D = P - last;
    lineErrorSpeed = (P * KP) + (D * KD);    
    last = P;   
    leftMotorSpeed = runSpeed + lineErrorSpeed;
    rightMotorSpeed = runSpeed - lineErrorSpeed; 
    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
  }
  motors.setSpeeds(0, 0);
  leftLed.turnOff();
  circuit.print(out);
}

void performRun(){
  bool done = false, ini = false;
  bool right_once = false, left_once = false, right_crossLine = false, left_crossLine = false, next_segment = false;
  rightLed.turnOn();
  last = 0;
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
  leftCount = 0;
  rightCount = 0;
  actualSpeed = runSpeed, actualDEV = runSpeed + 20;
  goalSpeed = runSpeed;
  circuitCounter = 0;
  aux_leftCount = 0;
  aux_rightCount = 0;
  
  while(!done){
    leftLinePosition = leftLightSensor.readLine(leftSensorValues);
    centerLinePosition = centerLightSensor.readLine(centerSensorValues);
    rightLinePosition = rightLightSensor.readLine(rightSensorValues);
 
    //RIGHT SENSOR
    if(rightLinePosition == 7001 && !right_once){ 
      right_once = true;
      if(!right_crossLine && straightLine()){
        if(!ini){
          music.playBeep();
          ini = true;
          nextSegment();
        }
        else{
          music.playBeep();
          done = true;
        }
      }
      else right_crossLine = false;
    }
    else if(rightLinePosition != 7001){
      right_once = false;
    }
    
    //LEFT SENSOR
    if(leftLinePosition == 7001 && !left_once){ 
      left_once = true;
      if(!left_crossLine){
        if(!next_segment){
          if(circuitCounter == circuit.getTam()){
            done = true;
          }
          else{
            music.playBeep();
            nextSegment();
          }
        }
        else next_segment = false;
      }
      else left_crossLine = false;
    }
    else if(leftLinePosition != 7001){
      left_once = false;
    }
    
    //CENTER SENSOR
    if(goalSpeed > runSpeed && ini){
      if(leftCount >= aux_leftCount * STRAIGHT_END && rightCount >= aux_rightCount * STRAIGHT_END){
        actualSpeed = runSpeed;
        actualDEV = actualSpeed + 20;
      }
      else if((leftCount >= aux_leftCount*STRAIGHT_BEGIN) && (rightCount >= aux_rightCount*STRAIGHT_BEGIN)){
        actualSpeed = goalSpeed;
        actualDEV = actualSpeed + 20;
      }
    }

    if(leftCount >= aux_leftCount && rightCount >= aux_rightCount && ini && circuitCounter < circuit.getTam()){
      next_segment = true;
      music.playBeep();
      nextSegment();
    }
       
    if(centerLinePosition == 0){
      leftMotorSpeed = -actualDEV;
      rightMotorSpeed = actualDEV;
    }
    else if(centerLinePosition == 7000){
       leftMotorSpeed = actualDEV;
       rightMotorSpeed = -actualDEV;
    }
    else if(centerLinePosition == 7001){
      leftMotorSpeed = actualSpeed;
      rightMotorSpeed = actualSpeed;
      left_crossLine = true;
      right_crossLine = true;
    }
    else{
        P = centerLinePosition - 3500;
        D = P - last;
        lineErrorSpeed = (P * KP) + (D * KD);    
        last = P;
        
        leftMotorSpeed = actualSpeed + lineErrorSpeed;
        rightMotorSpeed = actualSpeed - lineErrorSpeed; 
        if(leftMotorSpeed > actualDEV) leftMotorSpeed = actualDEV;
        else if(leftMotorSpeed < -actualDEV) leftMotorSpeed = -actualDEV;
        if(rightMotorSpeed > actualDEV) rightMotorSpeed = actualDEV;
        else if(rightMotorSpeed < -actualDEV) rightMotorSpeed = -actualDEV;
    }
    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
  }
  
  aux_rightCount = rightCount;
  last = 0;
  while(rightCount < aux_rightCount + 1500){
    centerLinePosition = centerLightSensor.readLine(centerSensorValues);
    P = centerLinePosition - 3500;
    D = P - last;
    lineErrorSpeed = (P * KP) + (D * KD);    
    last = P;   
    leftMotorSpeed = runSpeed + lineErrorSpeed;
    rightMotorSpeed = runSpeed - lineErrorSpeed; 
    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
  }
  motors.setSpeeds(0, 0);
  rightLed.turnOff();
}

void nextSegment(){
  leftCount = 0;
  rightCount = 0;
  aux_leftCount = circuit.getLeft(circuitCounter);
  aux_rightCount = circuit.getRight(circuitCounter);
  if(circuit.getSpeed(circuitCounter) > 1){
    goalSpeed = runSpeed * circuit.getSpeed(circuitCounter);
  }
  else{
    goalSpeed = runSpeed;
    actualDEV = runSpeed + 20;
  }
  actualSpeed = runSpeed;
  circuitCounter++;
}

bool straightLine(){
  return (2500 < centerLinePosition) && (centerLinePosition < 4500);
}

void debugSensors(){
  if(DEBUG){
    out.print("LEFT: ");
    out.print(leftLinePosition);
    out.print("   CENTER: ");
    out.println("LINE");
    out.print(P);
    out.print("     ");
    out.print(last);
    out.print("     ");
    out.println(lineErrorSpeed);
    out.print("   RIGHT: ");
    out.print(rightLinePosition);
    out.println();
    out.println();
    out.println("ENCODERS");
    out.print("Left: ");
    out.print(leftCount);
    out.print("   Right: ");
    out.println(rightCount);
    out.println();
    out.println();
    out.println("MOTORS");
    out.print("Left: ");
    out.println("MOTORS");
    out.print(leftMotorSpeed);
    out.print(" ");
    out.print("   Right: ");
    out.println(rightMotorSpeed);
    for(int i = 0; i < 3; i++) out.println();   
  } 
}

// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(encoder1_A) == HIGH) {
    if (digitalRead(encoder1_B) == LOW) {
      leftCount--;
      leftCount--;
    } else {
      leftCount++;
      leftCount++;
    }
  } else {
    if (digitalRead(encoder1_B) == LOW) {
      leftCount++;
      leftCount++;
    } else {
      leftCount--;
      leftCount--;
    }
  }
}

// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(encoder2_A) == HIGH) {
    if (digitalRead(encoder2_B) == LOW) {
      rightCount++;
      rightCount++;
    } else {
      rightCount--;
      rightCount--;
    }
  } else {
    if (digitalRead(encoder2_B) == LOW) {
      rightCount--;
      rightCount--;
    } else {
      rightCount++;
      rightCount++;
    }
  }
}

