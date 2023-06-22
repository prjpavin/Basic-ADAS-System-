// ADAS System using Arduino Mega, 5 Ultrasonic Sensors, Servo Motor, LCD Display, Keypad, and Motor Control

#include <LiquidCrystal.h>
#include <Servo.h>
#include <Keypad.h>
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(50, 51); // Example SoftwareSerial pins

// Pin Definitions
int cruisecounter = 0;
#define Break 4
#define SLAVE_ADDRESS 2
#define TRIGGER_PIN_FRONT_LEFT 2
#define ECHO_PIN_FRONT_LEFT 3
#define TRIGGER_PIN_FRONT_RIGHT 4
#define ECHO_PIN_FRONT_RIGHT 5
#define TRIGGER_PIN_REAR_CROSS 6
#define ECHO_PIN_REAR_CROSS 7
#define TRIGGER_PIN_LANE_LEFT 8
#define ECHO_PIN_LANE_LEFT 9
#define TRIGGER_PIN_LANE_RIGHT 10
#define ECHO_PIN_LANE_RIGHT 11
#define SERVO_PIN 12
#define RX_PIN 10    // Pin connected to the RX pin of HC-05
#define TX_PIN 11   
SoftwareSerial bluetooth(RX_PIN, TX_PIN);   // RX, TX
 // Pin connected to the TX pin of HC-05

// Motor Control Pins
const int enableAPin = 13;
const int motorA1Pin = 14;
const int motorA2Pin = 15;
const int enableBPin = 16;
const int motorB1Pin = 17;
const int motorB2Pin = 18;

// Distance thresholds (in centimeters)
#define SAFE_DISTANCE 50
#define WARNING_DISTANCE 100

// Lane-keeping constants
#define SERVO_CENTER_ANGLE 85
#define SERVO_MAX_LEFT_ANGLE 55
#define SERVO_MAX_RIGHT_ANGLE 120

// LCD Pins
#define LCD_RS 19
#define LCD_EN 20
#define LCD_D4 21
#define LCD_D5 22
#define LCD_D6 23
#define LCD_D7 24
#define LCD_COLS 16
#define LCD_ROWS 2
const int LED_pins[] = {25, 26, 27, 28}; 
const int LDR_pins[] = {A0, A1};

// Keypad Pins
const byte ROWS = 4; // Number of rows
const byte COLS = 4; // Number of columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {29, 30, 31, 32};    // Connect to the row pinouts of the keypad (example pins)
byte colPins[COLS] = {33, 34, 35, 36};   
#define RX_PIN 37    // Pin connected to the RX pin of HC-05
#define TX_PIN 38 // Connect to the column pinouts of the keypad (example pins)

const int headlightsPin = 39;   // Replace <headlights_pin_number> with the actual pin number for the headlights
const int hazardLightsPin = 40; 
const int initPin = 41;;  // Replace <hazard_lights_pin_number> with the actual pin number for the hazard lights
bool debugMode = false;
int prevDistances[6] = {0, 0, 0, 0, 0, 0};
int debug_counter = 0;

// Keypad Layout

// LCD object
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Global variables
long duration;
int distance;
Servo steeringServo;
int cruiseSpeed = 0;
bool hazardLightsOn = false;
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Wire.begin(SLAVE_ADDRESS); // Initialize I2C communication with slave address
  Wire.onReceive(receiveEvent);
  bluetooth.begin(9600); 
  mySerial.begin(9600);
  pinMode(13,OUTPUT);
  // Initialize serial communication
  Serial.begin(9600);
  pinMode(headlightsPin, OUTPUT);
pinMode(hazardLightsPin, OUTPUT);
  pinMode(initPin, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(LED_pins[i], OUTPUT);
  }

  // Set LDR pins as input
  for (int i = 0; i < 2; i++) {
    pinMode(LDR_pins[i], INPUT);
  }



  // Configure sensor pins
  pinMode(TRIGGER_PIN_FRONT_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_FRONT_RIGHT, INPUT);
  pinMode(TRIGGER_PIN_REAR_CROSS, OUTPUT);
  pinMode(ECHO_PIN_REAR_CROSS, INPUT);
  pinMode(TRIGGER_PIN_LANE_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LANE_LEFT, INPUT);
  pinMode(TRIGGER_PIN_LANE_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_LANE_RIGHT, INPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(enableAPin, OUTPUT);
  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(enableBPin, OUTPUT);
  pinMode(motorB1Pin, OUTPUT);
  pinMode(motorB2Pin, OUTPUT);

  // Attach steering servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER_ANGLE);
  systemInitializedBeep();

  // Initialize LCD display
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.setCursor(0, 0);
  lcd.print("ADAS System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  testservo();
  Serial.println("ADAS System Initializing...");

  

  // Wait for the LCD to initialize
  bool hazardLightsOn = false;

  // Clear LCD display
  lcd.clear();
}

void loop() {
  // Check for debug mode activation from serial monitor
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("debug")) {
      debugMode = !debugMode;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Debug Mode: ");
      lcd.print(debugMode ? "On" : "Off");
      Serial.print("Debug mode ");
      Serial.println(debugMode ? "activated!" : "deactivated!");
      debug_counter  = 0;
      delay(1000);
    }
  }

  if (debugMode) {
    printSensorData();
    if(debug_counter == 0){
    printAvailableRAM();
    debug_counter++;
    }
  }
  // Read distance from front-right ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT);
  if (distance < SAFE_DISTANCE) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Emergency Stop! ");
    lcd.setCursor(0, 1);
    lcd.print("Front");
    digitalWrite(13,0);
    
    Serial.println("Emergency action for Front sensor (forward collision avoidance)!");
    bluetooth.println("Emergency action for Front sensor (forward collision avoidance)!");
    playTeslaWarningSound();
  } else if (distance < WARNING_DISTANCE) {
    lcd.clear();
    digitalWrite(Break,HIGH);
    // Perform warning action for front-right sensor (adaptive cruise control)
    lcd.setCursor(0, 0);
    lcd.print("Warning!        ");
    lcd.setCursor(0, 1);
    lcd.print("Front");
    digitalWrite(13,0);
    delay(1000);
    digitalWrite(Break,LOW);
    
    cruise();
    Serial.println("Warning for Front sensor (adaptive cruise control)!");
    bluetooth.println("Warning for Front sensor (adaptive cruise control)!");
    playTeslaWarningSound();
  }

  // Read distance from rear-cross ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_REAR_CROSS, ECHO_PIN_REAR_CROSS);
  if (distance < SAFE_DISTANCE) {
    lcd.clear();
    digitalWrite(13,0);
     
    cruise();
    digitalWrite(Break,HIGH);
    lcd.setCursor(0, 0);
    lcd.print("Rear Cross Alert");
    lcd.setCursor(0, 1);
    lcd.print("Warning!");
    Serial.println("Emergency action for Rear Cross sensor (rear cross-traffic alert)!");
    bluetooth.println("Emergency action for Rear Cross sensor (rear cross-traffic alert)!");
    playTeslaWarningSound();
    delay(1000);
    digitalWrite(Break,LOW);
  }

  // Read distance from left ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_LANE_LEFT, ECHO_PIN_LANE_LEFT);
  if (distance < SAFE_DISTANCE) {
    lcd.clear();
    steeringServo.write(SERVO_MAX_RIGHT_ANGLE);
    lcd.setCursor(0, 0);
    lcd.print("Lane Departure ");
    lcd.setCursor(0, 1);
    digitalWrite(13,0);
     
    cruise();
    lcd.print("Warning!       ");
    Serial.println("Emergency action for Left sensor (lane-keeping)!");
    bluetooth.println("Emergency action for Left sensor (lane-keeping)!");
    delay(300);
    playTeslaWarningSound();
    steeringServo.write(85);
  } else if (distance < WARNING_DISTANCE) {
    lcd.clear();
    // Perform warning action for left sensor (lane-keeping)
    lcd.setCursor(0, 0);
    lcd.print("Lane Departure ");
    lcd.setCursor(0, 1);
    digitalWrite(13,0);
    lcd.print("Warning!       ");
    Serial.println("Warning for Left sensor (lane-keeping)!");
    bluetooth.println("Warning for Left sensor (lane-keeping)!");
    playTeslaWarningSound();
  }

  // Read distance from right ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_LANE_RIGHT, ECHO_PIN_LANE_RIGHT);
  if (distance < SAFE_DISTANCE) {
    lcd.clear();
       steeringServo.write(SERVO_MAX_LEFT_ANGLE);
    lcd.setCursor(0, 0);
    // Perform emergency action for right sensor (lane-keeping)
    lcd.setCursor(0, 0);
    lcd.print("Lane Departure ");
    lcd.setCursor(0, 1);
    lcd.print("Warning!       ");
    digitalWrite(13,0);
        delay(300);
    steeringServo.write(85);
     
    cruise();
    Serial.println("Emergency action for Right sensor (lane-keeping)!");
    bluetooth.println("Emergency action for Right sensor (lane-keeping)!");
    playTeslaWarningSound();
  } else if (distance < WARNING_DISTANCE) {
    lcd.clear();
   steeringServo.write(SERVO_MAX_LEFT_ANGLE);
    lcd.setCursor(0, 0);
    lcd.print("Lane Departure ");
    lcd.setCursor(0, 1);
    lcd.print("Warning!       ");
    Serial.println("Warning for Right sensor (lane-keeping)!");
    playTeslaWarningSound();
    digitalWrite(13,0);
    delay(300);
    steeringServo.write(85);
     
    cruise();
  }
  Wire.onReceive(receiveEvent);
    //Check for keypad input
  char key = keypad.getKey();
  if (key != NO_KEY) {
    handleKeypadInput(key);
  }
  int ldr1 = analogRead(LDR_pins[0]);
  int ldr2 = analogRead(LDR_pins[1]);

  if (ldr1 < 100 && ldr2 < 100) {
    // Turn on first two LEDs and turn off last two LEDs
    digitalWrite(LED_pins[0], HIGH);
    digitalWrite(LED_pins[1], HIGH);
    digitalWrite(LED_pins[2], LOW);
    digitalWrite(LED_pins[3], LOW);
  } else if (ldr1 > 900 && ldr2 > 900 && cruiseSpeed >= 80) {
    // Turn off first two LEDs and turn on last two LEDs
    digitalWrite(LED_pins[0], HIGH);
    digitalWrite(LED_pins[1], HIGH);
    digitalWrite(LED_pins[2], HIGH);
    digitalWrite(LED_pins[3], HIGH);
  } else {
    // Turn off all LEDs
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_pins[i], LOW);
    }
  }

  delay(100);  // Delay for stability
}


int readDistance(int triggerPin, int echoPin) {
  // Send ultrasonic pulse
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Measure the echo duration
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distance = duration * 0.034 / 2;

  // Return the distance
  return distance;
}

void handleKeypadInput(char key) {
  switch (key) {
    case '2':
      // Increase cruise speed
      cruiseSpeed += 10;
      if(cruiseSpeed>100)
      {
        lcd.clear();
        cruiseSpeed -= 10;
        lcd.setCursor(0,0);
        lcd.print("Max Cruise Speed");
        lcd.setCursor(0,1);
        lcd.print("Use Manual speed");
      }
      else{
      lcd.setCursor(0, 0);
      lcd.print("Cruise Speed:   ");
      lcd.setCursor(0, 1);
      lcd.print(cruiseSpeed);
      Serial.print("Cruise speed increased to: ");}
      Serial.println(cruiseSpeed);
      mySerial.write(cruiseSpeed+30);

      break;
    case '8':
      // Decrease cruise speed
      cruiseSpeed -= 10;
      if (cruiseSpeed < 0) {
        cruiseSpeed = 0;
      }
      lcd.setCursor(0, 0);
      lcd.print("Cruise Speed:   ");
      lcd.setCursor(0, 1);
      lcd.print(cruiseSpeed);
      Serial.print("Cruise speed decreased to: ");
      Serial.println(cruiseSpeed);
      mySerial.write(cruiseSpeed+30);
      break;
    case '5':
      // Activate/Deactivate cruise control
      if (cruisecounter) {
        cruisecounter = !cruisecounter;
        lcd.setCursor(0, 0);
        lcd.print("Cruise Control  ");
        lcd.setCursor(0, 1);
        lcd.print("Deactivated    ");
        Serial.println("Cruise control deactivated!");
        Wire.write('k');
        digitalWrite(13,LOW);
        turnMotorsOffB();
        turnMotorsOffA();
      } else {
        cruisecounter = !cruisecounter;
        lcd.setCursor(0, 0);
        lcd.print("Cruise Control  ");
        lcd.setCursor(0, 1);
        lcd.print("Activated      ");
        Serial.println("Cruise control activated!");
        Wire.write('/');
        //digitalWrite(13,HIGH);
        setMotorSpeedA(cruiseSpeed);
        setMotorSpeedB(cruiseSpeed);
      }
      break;
    case '4':
      // Turn on headlights
      digitalWrite(headlightsPin, HIGH);
      lcd.setCursor(0, 0);
      lcd.print("Headlights:     ");
      lcd.setCursor(0, 1);
      lcd.print("On");
      Serial.println("Headlights turned on!");
      break;
    case 'e':
      // Turn off headlights
      digitalWrite(headlightsPin, LOW);
      lcd.setCursor(0, 0);
      lcd.print("Headlights:     ");
      lcd.setCursor(0, 1);
      lcd.print("Off");
      Serial.println("Headlights turned off!");
      break;
    case '*':
      // Toggle hazard lights
      hazardLightsOn = !hazardLightsOn;
      digitalWrite(hazardLightsPin, hazardLightsOn ? HIGH : LOW);
      lcd.setCursor(0, 0);
      lcd.print("Hazard Lights:  ");
      lcd.setCursor(0, 1);
      lcd.print(hazardLightsOn ? "On " : "Off");
      Serial.println(hazardLightsOn ? "Hazard lights turned on!" : "Hazard lights turned off!");
      break;
    case 'A':
      // Set cruise speed
      lcd.setCursor(0, 0);
      lcd.print("Enter Speed:");
      lcd.setCursor(0, 1);
      lcd.blink();  // Enable cursor blinking to indicate input mode
      while (true) {
        char speedKey = keypad.getKey();
        if (speedKey == '*') {
          lcd.noBlink();  // Disable cursor blinking
          break;
        }
        if (speedKey >= '0' && speedKey <= '9') {
          cruiseSpeed = speedKey - '0';
          lcd.setCursor(0, 1);
          lcd.print(cruiseSpeed);
        }
      }
      lcd.setCursor(0, 0);
      lcd.print("Cruise Speed:   ");
      lcd.setCursor(0, 1);
      lcd.print(cruiseSpeed);
      Serial.print("Cruise speed set to: ");
      Serial.println(cruiseSpeed);
      break;
    case '6':
      // Toggle debug mode
      debugMode = !debugMode;
      lcd.clear();
      delay(100);
      lcd.setCursor(0, 0);
      lcd.print("Debug Mode: ");
      lcd.print(debugMode ? "On" : "Off");
      Serial.print("Debug mode ");
      Serial.println(debugMode ? "activated!" : "deactivated!");
      delay(3000);
      break;
    case 'z':
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connection established");
    lcd.setCursor(0,1);
    lcd.print("with UNO-2");
    delay(5000);
    Serial.println("Connection established with UNO-2");
    break;
    default:
      // Invalid key
      lcd.clear();
      delay(100);
      lcd.setCursor(0, 0);
      lcd.print("Invalid Key");
      lcd.setCursor(0, 1);
      lcd.print("              ");
      Serial.println("Invalid key pressed!");
      break;
  }
}

void printSensorData() {
  // Read and print distance from front-left ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_FRONT_LEFT, ECHO_PIN_FRONT_LEFT);
  Serial.print("Front-Left Sensor: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Read and print distance from front-right ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_FRONT_RIGHT, ECHO_PIN_FRONT_RIGHT);
  Serial.print("Front-Right Sensor: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Read and print distance from rear-cross ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_REAR_CROSS, ECHO_PIN_REAR_CROSS);
  Serial.print("Rear-Cross Sensor: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Read and print distance from left ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_LANE_LEFT, ECHO_PIN_LANE_LEFT);
  Serial.print("Left Sensor: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Read and print distance from right ultrasonic sensor
  distance = readDistance(TRIGGER_PIN_LANE_RIGHT, ECHO_PIN_LANE_RIGHT);
  Serial.print("Right Sensor: ");
  Serial.print(distance);
  Serial.println(" cm");
}
void printAvailableRAM() {
  lcd.clear();
  extern int __heap_start, *__brkval;
  int availableRAM;
  availableRAM = (int)&availableRAM - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);

  long freeStorage = getFreeStorage();

  lcd.setCursor(0, 0);
  lcd.print("Available RAM:");
  lcd.setCursor(0, 1);
  lcd.print(availableRAM);
  lcd.print(" bytes");
  delay(3000);

  lcd.setCursor(0, 2);
  lcd.print("Available Storage:");
  lcd.setCursor(0, 3);
  lcd.print(freeStorage);
  lcd.print(" bytes");
}

long getFreeStorage() {
  long size1 = 0xFFFF; // Set initial size to maximum value
  byte *buf;
  while ((buf = (byte *)malloc(size1)) == NULL) {
    // Try to allocate memory in progressively smaller chunks
    size1 /= 2;
    if (size1 == 0) {
      // No free memory available
      return 0;
    }
  }
  extern int __data_start;
  extern int *__brkval;
  int size = (int) &size - (__brkval == 0 ? (int) &__data_start : (int) __brkval);
  free(buf); // Free the allocated memory
  return size;
}


void testservo()
{
  int abc = 0;
  while(abc<(180+180))
  {
      for (int pos =0 ; pos <= 180; pos += 1,abc++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    steeringServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 1,abc++) { // goes from 180 degrees to 0 degrees
    steeringServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  }
  steeringServo.write(85);
}
void cruise(){
  if(cruisecounter){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Emergency");
  lcd.setCursor(0,1);
  lcd.print("Deactivated");
  Serial.println("Deactivated cruise control as warning or emergeny was raised");
}
}

void setMotorSpeedA(int speed) {
  // Ensure that the speed value is within the valid range (0-100)
  if (speed < 0) {
    speed = 0;
  } else if (speed > 100) {
    speed = 100;
  }

  // Map the speed value to a range suitable for PWM
  int pwmValue = map(speed, 0, 100, 0, 255);

  // Set the motor direction based on the speed value
  if (speed > 0) {
    digitalWrite(motorA1Pin, HIGH);
    digitalWrite(motorA2Pin, LOW);
  } else {
    digitalWrite(motorA1Pin, LOW);
    digitalWrite(motorA2Pin, LOW);
  }

  // Set the motor speed using PWM
  analogWrite(enableAPin, pwmValue);
}

void setMotorSpeedB(int speed) {
  // Ensure that the speed value is within the valid range (0-100)
  if (speed < 0) {
    speed = 0;
  } else if (speed > 100) {
    speed = 100;
  }

  // Map the speed value to a range suitable for PWM
  int pwmValue = map(speed, 0, 100, 0, 255);

  // Set the motor direction based on the speed value
  if (speed > 0) {
    digitalWrite(motorB1Pin, HIGH);
    digitalWrite(motorB2Pin, LOW);
  } else {
    digitalWrite(motorB1Pin, LOW);
    digitalWrite(motorB2Pin, LOW);
  }

  // Set the motor speed using PWM
  analogWrite(enableBPin, pwmValue);
}
void turnMotorsOffA() {
  digitalWrite(motorA1Pin, LOW);
  digitalWrite(motorA2Pin, LOW);
}

void turnMotorsOffB() {
  digitalWrite(motorB1Pin, LOW);
  digitalWrite(motorB2Pin, LOW);
}
void receiveEvent(int numBytes) {
  while (Wire.available()) {
    char key = Wire.read();
    // Process received character as needed
    Serial.print("Received character: ");
    bluetooth.print("Received character: ");
    Serial.println(key);
    handleKeypadInput(key);
   
  }
}
void playTeslaWarningSound() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(initPin, HIGH);   // Turn on the warning buzzer
    delay(250);                       // Beep duration
    digitalWrite(initPin, LOW);    // Turn off the warning buzzer
    delay(250);                       // Delay between beeps
  }
  delay(500);                          
}
void systemInitializedBeep() {
 for (int i = 0; i < 2; i++) {
    digitalWrite(initPin, HIGH);     // Turn on the initialization buzzer
    delay(100);                      // Beep duration
    digitalWrite(initPin, LOW);      // Turn off the initialization buzzer
    delay(100);                      // Delay between beeps
  }
}