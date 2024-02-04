// Tasks schedule 
#include <ExampleConstants.h>
#include <SchedBase.h>
#include <SchedTask.h>

#include <IRremote.hpp>
#define IR_RECEIVE_PIN 2
int remote_btn;

//dust sensor
const int dustPin = A12;

//Buttons
const int startButtonPin = 3;  // Push button for starting the program
const int stopButtonPin = 4;   // Push button for stopping the machine

//Motors
const int motor1Pin1 = 5;  // Motor1 input 1 pin (motion)
const int motor1Pin2 = 6;  // Motor1 input 2 pin (motion)
const int motor2Pin1 = 7;  // Motor2 input 1 pin (motion)
const int motor2Pin2 = 8;  // Motor2 input 2 pin (motion)
const int brushPin1 = 9;  // Brush input 1 pin 
const int brushPin2 = 10;  // Brush input 2 pin 

//Sensors
const int psr1Pin = 13;        // Proximity sensor right 1
const int psl1Pin = 15;        // Proximity sensor left 1
const int en_S = 17;

int psr1State = digitalRead(psr1Pin);
int psl1State = digitalRead(psl1Pin);

//LEDs
const int startLED = A0;  // LED for start
const int dryLED = A1;    // LED for moving to the right
const int wetLED = A2;    // LED for moving to the left
const int stopLED = A3;   // LED for stopping

//Pump
const int pump=A4; // Pump relay 

//Methods schedule
void IR_recev(); // Remote control
void cleaningProcess();
void startCleaning();
void stopMotors();
void restartCleaning();
int dustSensor(int dustPin);

SchedTask startC(0,1000,startCleaning); //  Define the start cleaning (dispatch now, every 1 sec)
SchedTask stopCleaning(1000,1000,stopMotors); // Define the stop cleaning (dispatch after 1 sec, every 1 sec)
SchedTask restartC(2000,1000,restartCleaning); 


class MotorDriver {
public:
  MotorDriver(int pin1, int pin2, int pin3) : pin1(pin1), pin2(pin2),pin3(pin3){
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);

  }

  void moveForward() {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    digitalWrite(en_S, HIGH);
    
  }

  void moveBackward() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    digitalWrite(en_S, HIGH);
  }

  void Break() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(en_S, LOW);
  }

private:
  int pin1;
  int pin2;
  int pin3;
};

MotorDriver motor1(motor1Pin1, motor1Pin2, en_S);
MotorDriver motor2(motor2Pin1, motor2Pin2, en_S);
MotorDriver brush(brushPin1, brushPin2, en_S);

void moveLeft() {
  Serial.println("Moving Left");
  motor1.moveForward();
  motor2.moveBackward();
  brush.moveForward(); 
  digitalWrite(pump,HIGH);
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, HIGH);
  digitalWrite(stopLED, LOW);
}

void moveRight() {
  Serial.println("Moving Right");
  motor1.moveBackward();
  motor2.moveForward();
  brush.moveBackward();
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, HIGH);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, LOW);
}

void stopMotors() {
  IR_recev();
  if (digitalRead(stopButtonPin) == LOW || remote_btn==2) {
    Serial.println("Stopping Motors");
    motor1.Break();
    motor2.Break();
    brush.Break();
    digitalWrite(pump,LOW);
    digitalWrite(startLED, LOW);
    digitalWrite(dryLED, LOW);
    digitalWrite(wetLED, LOW);
    digitalWrite(stopLED, HIGH);
  }
}

void restartCleaning() {
  if (digitalRead(stopLED) == HIGH && (psr1State == LOW || (psr1State == HIGH && psl1State == HIGH)) && (remote_btn==1 || digitalRead(startButtonPin) == LOW)) {
        while (psl1State == HIGH) {
          Serial.println("Restarting to Left");
          motor1.moveForward();
          motor2.moveBackward();
          digitalWrite(startLED,HIGH);
          digitalWrite(dryLED, LOW);
          digitalWrite(wetLED, LOW);
          digitalWrite(stopLED, HIGH);
          // Update sensor states inside the loop
          psl1State = digitalRead(psl1Pin);
          delay(100);
        }
        
      cleaningProcess();
  }
}

void IR_recev() {

  remote_btn = 0;

  if (IrReceiver.decode()) {
      //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data 
      //byte remotepin=IrReceiver.decodedIRData.decodedRawData;
      switch (IrReceiver.decodedIRData.decodedRawData) {
        case 0xBA45FF00: // start button pressed which is 1 in HEX for example
        //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
        Serial.println("start");
        remote_btn = 1;
        break;

        case 0xB946FF00: // stop button pressed which is 2 in HEX for example
        //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); 
        Serial.println("stop");
        remote_btn = 2;
        break;
      }
      IrReceiver.resume(); // Enable receiving of the next value
  }
}

void cleaningProcess() {
    
    // Check if conditions are met to move to the right
    if (psl1State == LOW) {
      // Move to the right until certain conditions are met
        while (psr1State == HIGH) {
          moveRight();
          // Update sensor states inside the loop
          psr1State = digitalRead(psr1Pin);
          delay(100);
        }
    }
    
    // Check if conditions are met to move to the left
    if (psr1State == LOW && psl1State == HIGH) {

      // Move to the left until certain conditions are met
        while (psl1State == HIGH) {
          moveLeft();
          // Update sensor states inside the loop
          psl1State = digitalRead(psl1Pin);
          delay(100);
      }
    }

    motor1.Break();
    motor2.Break();
    brush.Break();
    digitalWrite(pump,LOW);
    digitalWrite(startLED, LOW);
    digitalWrite(wetLED, LOW);
    digitalWrite(stopLED, HIGH);
}
  
void startCleaning() {
  // Read the state of the stop button and sensor states
  IR_recev();

  int dustSignal = dustSensor(dustPin);

  // Check if the start button is pressed
  if (psl1State == LOW && (digitalRead(startButtonPin) == LOW || remote_btn == 1 || dustSignal == 1)) {
    
    cleaningProcess();

  }
}


int dustSensor(int dustPin) {

int dustRatio = analogRead(dustPin);

if (dustRatio >= 20) {
  while(dustRatio >= 20) {
}
return 1;
}
return 0;
}

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  pinMode(startButtonPin, INPUT);
  pinMode(stopButtonPin, INPUT);
  pinMode(pump, OUTPUT);
  pinMode(startLED, OUTPUT);
  pinMode(dryLED, OUTPUT);
  pinMode(wetLED, OUTPUT);
  pinMode(stopLED, OUTPUT);
}

void loop() {

  SchedBase::dispatcher();

}
