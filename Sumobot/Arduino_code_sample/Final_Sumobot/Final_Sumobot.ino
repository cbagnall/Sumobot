//SUMOBOT Competition 2014/2015 

//authors: Cody, Clayton, Maaz

const unsigned int leftFwd = 3;       // PWM forward pin
const unsigned int leftBwd = 11;
const unsigned int rightFwd = 5;
const unsigned int rightBwd = 6;      // PWM reverse pin
const unsigned int frontLeft = A5;    // front left sensor
const unsigned int frontRight = A3;   // front right sensor
const unsigned int backLeft = A4;     // back left sensor
const unsigned int backRight = A0;    // back right sensor
const unsigned int Sensor = A1;       // Ultrasonic sensor
const unsigned int threshold = 150;    
const unsigned int Sensor_thresh = 20;

#include <Servo.h>                     // import servo library and functions
Servo servo1;
int servopin=2;

void setup() {
                                      // specifies baudrate for serial comm (Allows communication with robot)
  Serial.begin(9600);
  
                                      // initiallize pins as inputs
  pinMode(frontLeft, INPUT);
  pinMode(frontRight, INPUT);
  pinMode(backLeft, INPUT);
  pinMode(backRight, INPUT);
  pinMode(IRSensor, INPUT);
  
  pinMode(leftFwd, OUTPUT);
  pinMode(leftBwd, OUTPUT);
  pinMode(rightFwd, OUTPUT);
  pinMode(rightBwd, OUTPUT);
  servo1.attach(servopin);
  
  unsigned int FL = analogRead(frontLeft);    // initilization to make sure robot is not partly inside/outside ring to begin. Initializes by going fwd
  unsigned int FR = analogRead(frontRight);
  unsigned int BL = analogRead(backLeft);
  unsigned int BR = analogRead(backRight);
  if (FL < threshold && FR < threshold && BL < threshold && BR < threshold) {    //if no white lines detected move forward (there were ultimately 4 white line sensors implimented)
    servo1.write(170);
    delay(10000);
  }
    delay(4300);                    // move slowly
    servo1.write(70);
  
}

void loop(){                      //main loop checks that robot is inside circle then scans for other robot --> basic algorithm is check rign, check opponent, move
    checkCircle();
    checkOpponent();
}

void checkCircle() {                               // checks all 4 black/white IR sensors             
  unsigned int fl = analogRead(frontLeft);
  unsigned int fr = analogRead(frontRight);
  unsigned int bl = analogRead(backLeft);
  unsigned int br = analogRead(backRight);
    
  if ((fl < threshold) || (fr < threshold)) {      // checks for white lines in front and reverses/turns if it sees any
     powerBwd();
     delay(200);
     clockwise();
     delay(1000);
     powerFwd();
  }
   
  else if ((bl < threshold) || (br < threshold)) { // checks for white lines behind drives fwd if it sees any
    powerFwd();
    delay(500);
  }
       
}

void checkOpponent(){
  unsigned int reading = analogRead(Sensor);
  if (reading > Sensor_thresh){
    powerFwd();
    while (reading > Sensor_thresh){    // inner loop if it sees opponent
      reading = analogRead(Sensor);
      unsigned int FL = analogRead(frontLeft);
      unsigned int FR = analogRead(frontRight);
      if(FL < threshold || FR < threshold){    // white line detection as it pushes opponent out of circle
        powerBwd();
        delay(200);
        clockwise();
        delay(1000);
        powerFwd();
        checkCircle();
      }
    }
    unsigned int r1 = analogRead(Sensor);      // three quick reads to ensure no false positives from sensor
    unsigned int r2 = analogRead(Sensor);
    unsigned int r3 = analogRead(Sensor);
    if(r1>Sensor_thresh && r2>Sensor_thresh && r3>Sensor_thresh){
      checkOpponent();
    }
    else{
      powerFwd();        // move forwards if there is no obstacle or opponent
    }
  }
}

void setMotor(unsigned int LFwd, unsigned int LBwd, unsigned int RFwd, unsigned int RBwd){    // motor is controlled via H-bridge
  analogWrite(leftFwd, LFwd);
  analogWrite(leftBwd, LBwd);
  analogWrite(rightFwd, RFwd);
  analogWrite(rightBwd, RBwd);
}

void clockwise(){setMotor(255,0,0,255);}                //function definitions for movement
void counterClockwise() {setMotor(0,255,255,0);}
void forward(){setMotor(105,0,100, 0);}
void backward(){setMotor(0,105,0,100);}
void powerFwd(){setMotor(235,0,255,0);}
void powerBwd(){setMotor(0,235,0,255);}

