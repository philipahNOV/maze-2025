#include "Adafruit_NeoPixel.h"
#include <Servo.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// NeoPixel settings
#define PIN            30
#define NUMPIXELS      10
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 1; // delay for half a second

// Define PWM pins
const int pwmPin1 = 9;  // Pin 9 for motor 1 direction 1
const int pwmPin2 = 10; // Pin 10 for motor 1 direction 2
const int pwmPin3 = 11; // Pin 11 for motor 2 direction 1
const int pwmPin4 = 12; // Pin 12 for motor 2 direction 2

const int X_pin = A0; // analog pin connected to X output
const int Y_pin = A1; // analog pin connected to Y output

const int feedbackPin1 = A4; // Analog input for Motor 1 feedback
const int feedbackPin2 = A3; // Analog input for Motor 2 feedback

const int minFeedbackValue = 150; // Minimum allowed feedback value (limit)
const int maxFeedbackValue = 750; // Maximum allowed feedback value (limit)

const int minAllowed_Velocity = 1;

int Mid1 = -1;
int Mid2 = -1;

int x_r = -1;
int y_r = -1;

float const K_p = 1.5;
const int x_Level_Gain = 40;
const int y_Level_Gain = 20;

int elManuel = 0; // Manual mode flag

Servo myServo;
const int servoPin = 2;
const int posLow = 135;
const int posHigh = 25;
int currentPosition = posLow;

int elevator = 0; // Elevator mode flag

int dirMotor1 = -2;   // Direction for Motor 1
int dirMotor2 = -2;   // Direction for Motor 2
int speedMotor1 = 0; // Speed for Motor 1
int speedMotor2 = 0; // Speed for Motor 2

int x_pos, y_pos, x_ref, y_ref, x_error, y_error, motor_vel_x, motor_vel_y, x_s, y_s;
// Function to set motor speed and direction
void setMotorSpeed(int dirMotor1, int dirMotor2, int speedMotor1, int speedMotor2) {
  // Map new direction values to existing ones
  if (dirMotor1 == 1) dirMotor1 = -1; //Går oppover
  if (dirMotor1 == 2) dirMotor1 = -2; //Står stille
  if (dirMotor1 == 3) dirMotor1 = -3; //Går nedover

  if (dirMotor2 == 1) dirMotor2 = -1;
  if (dirMotor2 == 2) dirMotor2 = -2;
  if (dirMotor2 == 3) dirMotor2 = -3;


  if (dirMotor1 == -1 && analogRead(feedbackPin1) < maxFeedbackValue) {
    // Up
    analogWrite(pwmPin1, speedMotor1);
    analogWrite(pwmPin2, 0);
    
  } else if (dirMotor1 == -3 && analogRead(feedbackPin1) > minFeedbackValue) {
    // Down
    analogWrite(pwmPin1, 0);
    analogWrite(pwmPin2, speedMotor1);
  } else{
    // Full stop
    analogWrite(pwmPin1, 0);
    analogWrite(pwmPin2, 0);
  }

  // Process Motor 2 direction and speed
  if (dirMotor2 == -1 && analogRead(feedbackPin2) < maxFeedbackValue) {
    // Up
    analogWrite(pwmPin3, speedMotor2);
    analogWrite(pwmPin4, 0);
  } else if (dirMotor2 == -3 && analogRead(feedbackPin2) > minFeedbackValue) {
    // Down
    analogWrite(pwmPin3, 0);
    analogWrite(pwmPin4, speedMotor2);
  } else{
    // Full stop
    analogWrite(pwmPin3, 0);
    analogWrite(pwmPin4, 0);
  }
}

void moveToPosition(int targetPosition, int speed=10) {
  if (currentPosition < targetPosition) {
    for (int pos = currentPosition; pos <= targetPosition; pos++) {
      myServo.write(pos);
      delay(speed);
    }
  } else {
    for (int pos = currentPosition; pos >= targetPosition; pos--) {
      myServo.write(pos);
      delay(speed);
    }
  }
  currentPosition = targetPosition;
}

void setup() {
  // NeoPixel setup
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  pixels.setBrightness(255);
  pixels.begin(); // This initializes the NeoPixel library.

  // Set PWM pins as outputs
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);

  myServo.attach(servoPin);
  myServo.write(currentPosition);
  delay(5000);

  for(int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 255)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }

  Serial.begin(9600);
}

void loop() {
  setMotorSpeed(dirMotor1, dirMotor2, speedMotor1, speedMotor2);
  // Motor control logic
  if (elevator == 0){
    if (Serial.available() >= 4 * sizeof(int)) {
      dirMotor1 = Serial.parseInt();   
      dirMotor2 = Serial.parseInt();   
      speedMotor1 = Serial.parseInt(); 
      speedMotor2 = Serial.parseInt(); 

      while (Serial.available() && Serial.read() != '\n') {} // Clear the buffer

      if (dirMotor1 == 100 && dirMotor2 == 100 && speedMotor1 == 100 && speedMotor2 == 100) {
          elManuel = 1;
      } else if (dirMotor1 == 200 && dirMotor2 == 200 && speedMotor1 == 200 && speedMotor2 == 200) {
          elManuel = 0;
          dirMotor1 = -2; 
          dirMotor2 = -2;  
          speedMotor1 = 0; 
          speedMotor2 = 0;
      } else if (dirMotor1 == 300 && dirMotor2 == 300 && speedMotor1 == 300 && speedMotor2 == 300)
      {
        elevator = 1;
      }

    else if (dirMotor1 >= 120 && dirMotor1 <= 124 &&
        dirMotor2 >= 120 && dirMotor2 <= 124 &&
        speedMotor1 == 120 && speedMotor2 == 120) {
          
        if (dirMotor1 == 120) {
            // Begge aktuatorene skal stå st ille
            dirMotor1 = -2;
            speedMotor1 = 0;
        } 
       else if (dirMotor1 == 123) {
            // x-aktuator skal øke
            dirMotor1 = -1;
            speedMotor1 = minAllowed_Velocity + x_Level_Gain;
        }
        else if (dirMotor1 == 124) {
            // x skal synke
            dirMotor1 = -3;
            speedMotor1 = minAllowed_Velocity + x_Level_Gain;
        }
        
        if (dirMotor2 == 120) {
            // Stå stille
            dirMotor2 = -2;
            speedMotor2 = 0;
        } 
       else if (dirMotor2 == 123) {
            // Øke y-aktuatoren
            dirMotor2 = -1;
            speedMotor2 = minAllowed_Velocity + y_Level_Gain;
        }
        else if (dirMotor2 == 124) {
            // Senke y-aktuatoren
            dirMotor2 = -3;
            speedMotor2 = minAllowed_Velocity + y_Level_Gain;
        }

    }
    else if (dirMotor1 == 777 && dirMotor2 == 777 && speedMotor1 == 777 && speedMotor2 == 777) {
        Mid1 = analogRead(feedbackPin1);
        Mid2 = analogRead(feedbackPin2);
        //for å lagre midtpunkt på arduino
    }

      
      else if (speedMotor1 == 201 && speedMotor2 == 799){
        //P-regulator
        x_pos = analogRead(feedbackPin1);
        y_pos = analogRead(feedbackPin2);
        
        x_ref = dirMotor1 + Mid1;
        y_ref = dirMotor2 + Mid2;
        x_r = constrain(x_ref, minFeedbackValue, maxFeedbackValue);
        y_r = constrain(y_ref, minFeedbackValue, maxFeedbackValue);
        
        x_error = x_r - x_pos;
        y_error = y_r - y_pos; //jetson (dirmotor eller speedmotor) - analog read??

        motor_vel_x = K_p * x_error;
        motor_vel_y = K_p * y_error;

        
        x_s = constrain(motor_vel_x, -255, 255);
        y_s = constrain(motor_vel_y, -255, 255); //Ikke 0 pga jeg fikser det med abs senere

        if (abs(x_s) < minAllowed_Velocity){ //Pga aktuatoren ikke godkjente de minste fartene
          dirMotor1 = -2; 
          speedMotor1 = 0; 
        }
        else if (x_error > 0){
          dirMotor1 = -1;
          speedMotor1 = abs(x_s); 
        }
        else if (x_error < 0){ 
          dirMotor1 = -3; //problem?????
          speedMotor1 = abs(x_s); 
        }
        if (abs(y_s) < minAllowed_Velocity){ //Pga aktuatoren ikke godkjente de minste fartene
          dirMotor2 = -2; 
          speedMotor2 = 0; 
        }
        else if (y_error > 0){
          dirMotor2 = -1;
          speedMotor2 = abs(y_s); 
        }
        else if (y_error < 0){ 
          dirMotor2 = -3; //problem?????
          speedMotor2 = abs(y_s); 
        }
        
      }
      
    }
    

    if (elManuel == 1) {
      int x_speed = analogRead(X_pin);
      int y_speed = analogRead(Y_pin);

      // Constrain the analog input values to be between 0 and 1000
      x_speed = constrain(x_speed, 2, 999);
      y_speed = constrain(y_speed, 2, 999);

      int x_dir = -2; // Default to stop
      int y_dir = -2; // Default to stop
      
      int max_speed = 255;

      if (x_speed >= 550) {
          x_speed = map(x_speed, 550, 1000, 1, max_speed);
          x_dir = -1;
      } else if (x_speed <= 450) {
          x_speed = map(x_speed, 450, 1, 1, max_speed);
          x_dir = -3;
      } else {
          x_speed = 0;
      }

      if (y_speed >= 550) {
          y_speed = map(y_speed, 550, 1000, 1, max_speed);
          y_dir = -3;
      } else if (y_speed <= 450) {
          y_speed = map(y_speed, 450, 1, 1, max_speed);
          y_dir = -1;
      } else {
          y_speed = 0;
      }

      dirMotor1 = x_dir;
      dirMotor2 = y_dir;
      speedMotor1 = x_speed;
      speedMotor2 = y_speed;
    }

  }else{
    if (elevator == 1){
      delay(1000);
      moveToPosition(posLow);
      delay(1000); // 5 seconds to make sure the ball has had the time to fall into possition. 
      elevator = 2;
    } else if (elevator == 2){
      if (analogRead(feedbackPin1) > 620){
        dirMotor1 = -3;
        dirMotor2 = -2;
        speedMotor1 = 100;
        speedMotor2 = 0;
      } else if (analogRead(feedbackPin1) < 600)
      {
        dirMotor1 = -1;
        dirMotor2 = -2;
        speedMotor1 = 100;
        speedMotor2 = 0;
      } else if (analogRead(feedbackPin2) > 620)
      {
        dirMotor1 = -2;
        dirMotor2 = -3;
        speedMotor1 = 0;
        speedMotor2 = 100;
      }else if (analogRead(feedbackPin2) < 600)
      {
        dirMotor1 = -2;
        dirMotor2 = -1;
        speedMotor1 = 0;
        speedMotor2 = 100;
      } else {
        dirMotor1 = -2; 
        dirMotor2 = -2;  
        speedMotor1 = 0; 
        speedMotor2 = 0;
        
        elevator = 3;
      }

    } else if (elevator == 3)
    {
      delay(1000);
      moveToPosition(posHigh);
      delay(2000);
      elevator = 4;
    } else if (elevator == 4)
    {
      if (analogRead(feedbackPin2) > 150){
        dirMotor1 = -2;
        dirMotor2 = -3;
        speedMotor1 = 0;
        speedMotor2 = 255;
      }else{
        dirMotor1 = -2; 
        dirMotor2 = -2;  
        speedMotor1 = 0; 
        speedMotor2 = 0;
        elevator = 5;
      }
    } else if (elevator == 5)
    {
      delay(1000);
      moveToPosition(posLow);
      delay(2000);
      elevator = 0;
    }
  }
}