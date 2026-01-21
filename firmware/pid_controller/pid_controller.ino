/*
   7MRI0060 - Applied Medical Robotics Module
   October 2024
   Author: Alejandro Granados and Harry Robertshaw

   Purpose: Run a PID controller

   Tasks:
    1. Set pins to code​
    2. Compute degrees and bound them to [-360,360]​
    3. Compute elapsed time (deltaT) and control the frequency of printing​
    4. Set demand position in degrees and compute errors (compared to previous, accumulation, and differential). 
       Hint: make sure you update previous error at the end.
    5. Compute PID​
    6. Send voltage to motors​
    7. Plot demand versus current position in degrees
*/

// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR = 3575.0855;
const float GR = 297.924;
const float CPR = 3;

// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = A3;
const int encoderPinB_m1 = A2;

const int encoderPinA_m2 = A1;
const int encoderPinB_m2 = 11;

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;

// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

// Variables for encoder positions and desired positions
long currentPosition_m1 = 0;
long currentPosition_m2 = 0;
float demandPositionInDegrees_m1 = 90.0;
float demandPositionInDegrees_m2 = 90.0;
float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;
float windupGuard=70;

// PID gains
// current best: 55,0,1
float Kp_m1 = 55, Ki_m1 = 0, Kd_m1 = 1;
float Kp_m2 = 55, Ki_m2 = 0, Kd_m2 = 1;

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

// Global variables
int i = 0;                // counter
String matlabStr = "";    // receives the string from matlab, it is empty at first
bool readyToSend = false; // flag to indicate a command was received and now ready to send back to matlab

char c;                   // characters received from matlab
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab
int offset1 = 0.0;
int offset2 = 0.0;

void setup() {
  Serial.begin(9600);

  // Task 1: Initialize the pins using pinMode and attachInterrupt functions
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);

  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);

  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);
  delay(3000);

  previousTime = micros();
}

void loop() {
  // Receive and parse data from MATLAB
  if (readyToSend == false) {
    if (Serial.available() > 0) {
      c = Serial.read();
      matlabStr += c;

      if (matlabStr.indexOf(";") != -1) {
        readyToSend = true;
        
        int posComma1 = matlabStr.indexOf(",");
        val1 = matlabStr.substring(1, posComma1).toFloat();
        int posEnd = matlabStr.indexOf(";");
        val2 = matlabStr.substring(posComma1 + 1, posEnd).toFloat();
        
        matlabStr = "";  // Clear string after parsing
        demandPositionInDegrees_m1 = val1 + offset1;
        demandPositionInDegrees_m2 = val2 + offset2;
      }
    }
  }
  
  // Task 2: Compute the current position in degrees and bound it to [-360,360]
  currentPositionInDegrees_m1 = ((counter_m1 * 360) / (CPR * GR * 2));
  currentPositionInDegrees_m2 = ((counter_m2 * 360) / (CPR * GR * 2));
  
  if (currentPositionInDegrees_m1 >= 360 || currentPositionInDegrees_m1 <= -360){
    counter_m1 -= ((CPR * GR * 2) * ((int)(currentPositionInDegrees_m1 / 360)));
  }
  if (currentPositionInDegrees_m2 >= 360 || currentPositionInDegrees_m2 <= -360){
    counter_m2 -= ((CPR * GR * 2) * ((int)(currentPositionInDegrees_m2 / 360)));
  }

  // Task 3: Compute elapsed time (deltaT) and control the frequency of printing​
  currentTime = micros();
  deltaT = currentTime - previousTime;
  previousTime = currentTime;

if (deltaT > 400) {
    // Task 4: Compute error (P,I,D), and ensure that the previous error is updated
    // Calculate error for motor 1
    float errorPositionInDegrees_m1 = currentPositionInDegrees_m1 - demandPositionInDegrees_m1; 
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT; // Convert microseconds to seconds

    // Update the integral term with windup prevention
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1 * deltaT;
    // Update integral in seconds

    // Optionally clamp the integral term to prevent windup
    errorPositionInDegrees_sum_m1 = constrain(errorPositionInDegrees_sum_m1, -windupGuard, windupGuard);

    // Update previous error
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;

    // Calculate error for motor 2
    float errorPositionInDegrees_m2 = currentPositionInDegrees_m2 - demandPositionInDegrees_m2; 
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT; // Convert microseconds to seconds

    // Update the integral term for motor 2 with windup prevention
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2 * deltaT; // Update integral in seconds
    errorPositionInDegrees_sum_m2 = constrain(errorPositionInDegrees_sum_m2, -windupGuard, windupGuard);

    // Update previous error
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

    // Task 5: Compute the PID output for motor 1
    float controllerOutput_m1 = (Kp_m1 * errorPositionInDegrees_m1) + 
                                 (Ki_m1 * errorPositionInDegrees_sum_m1) + 
                                 (Kd_m1 * errorPositionInDegrees_diff_m1);
    controllerOutput_m1 = constrain(controllerOutput_m1, -255, 255);

    // Compute the PID output for motor 2
    float controllerOutput_m2 = (Kp_m2 * errorPositionInDegrees_m2) + 
                                 (Ki_m2 * errorPositionInDegrees_sum_m2) + 
                                 (Kd_m2 * errorPositionInDegrees_diff_m2);
    controllerOutput_m2 = constrain(controllerOutput_m2, -255, 255);

    // Task 6: Send voltage to 
    if (controllerOutput_m1 > 0) {
      digitalWrite(motorPin1_m1, HIGH);
      digitalWrite(motorPin2_m1, LOW);
      analogWrite(enablePin_m1, controllerOutput_m1);
    } else {
      digitalWrite(motorPin1_m1, LOW);
      digitalWrite(motorPin2_m1, HIGH);
      analogWrite(enablePin_m1, -controllerOutput_m1);
    }

    if (controllerOutput_m2 > 0) {
      digitalWrite(motorPin1_m2, HIGH);
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(enablePin_m2, controllerOutput_m2);
    } else {
      digitalWrite(motorPin1_m2, LOW);
      digitalWrite(motorPin2_m2, HIGH);
      analogWrite(enablePin_m2, -controllerOutput_m2);
    }
  }

  if (readyToSend)  // arduino has received command form matlab and now is ready to send
  {
    // e.g. c1,100,100
    Serial.print("c");                    // command
    Serial.print(i);                      // series 1
    Serial.print(",");                    // delimiter
    Serial.print(currentPositionInDegrees_m1);
    Serial.print(",");
    Serial.print(currentPositionInDegrees_m2); // series 2
    Serial.write(13);                     // carriage return (CR)
    Serial.write(10);                     // new line (NL)
    i += 1;
  }
}

// Interrupt functions for tracking the encoder positions
void updateEncoder_m1() {
  // Code to update counter_m1 based on the state of the encoder pins
  // Read current states of channels A and B
  int aState_m1 = digitalRead(encoderPinA_m1);
  int bState_m1 = digitalRead(encoderPinB_m1);

  // Check if the state of channel A has changed
  if (aState_m1 != aLastState_m1) {
    // Determine the direction of rotation by comparing A and B states
    if (aState_m1 != bState_m1) {
      counter_m1++;  // Clockwise rotation
    } else {
      counter_m1--;  // Counterclockwise rotation
    }
  }

  // Update the last known state of channel A
  aLastState_m1 = aState_m1;
}

void updateEncoder_m2() {
  // Code to update counter_m2 based on the state of the encoder pins
  // Read current states of channels A and B
  int aState_m2 = digitalRead(encoderPinA_m2);
  int bState_m2 = digitalRead(encoderPinB_m2);

  // Check if the state of channel A has changed
  if (aState_m2 != aLastState_m2) {
    // Determine the direction of rotation by comparing A and B states
    if (aState_m2 != bState_m2) {
      counter_m2++;  // Clockwise rotation
    } else {
      counter_m2--;  // Counterclockwise rotation
    }
  }

  // Update the last known state of channel A
  aLastState_m2 = aState_m2;
}
