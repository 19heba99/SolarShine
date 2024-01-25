//Tasks schedule 
#include <ExampleConstants.h>
#include <SchedBase.h>
#include <SchedTask.h>

#include <IRremote.hpp>
#define IR_RECEIVE_PIN 2
int remote_btn;

//Buttons
const int startButtonPin = 3;  // Push button for starting the program
const int stopButtonPin = 4;   // Push button for stopping the machine

//Motors
const int motor1Pin1 = 5;  // Motor1 input 1 pin (motion)
const int motor1Pin2 = 6;  // Motor1 input 2 pin (motion)
const int motor1Pin3=7;
const int motor1Pin4=8;

//const int motor2Pin1 = 7;  // Motor2 input 1 pin (motion)
//const int motor2Pin2 = 8;  // Motor2 input 2 pin (motion)
const int brushPin1 = 9;  // Brush input 1 pin 
const int brushPin2 = 10;  // Brush input 2 pin 

//Sensors
const int irlPin = 11;          // IR sensor left
const int irrPin = 12;          // IR sensor right
const int psr1Pin = 13;        // Proximity sensor right 1
const int psr2Pin = 14;        // Proximity sensor right 2
const int psl1Pin = 15;        // Proximity sensor left 1
const int psl2Pin = 16;        // Proximity sensor left 2

//LEDs
const int startLED = A0;  // LED for start
const int dryLED = A1;    // LED for moving to the right
const int wetLED = A2;    // LED for moving to the left
const int stopLED = A3;   // LED for stopping

//Pump
const int pump=A4; // Pump relay 

//Methods schedule
int IR_recev(); // Remote control
void startCleaning();
void stopMotors();

SchedTask startC(0,500,startCleaning); //  Define the start cleaning (dispatch now, every 1 sec)
SchedTask stopCleaning(0,500,stopMotors); // Define the stop cleaning (dispatch now, every 1 sec)


class MotorDriver {
public:
  MotorDriver(int pin1, int pin2, int pin3, int pin4) : pin1(pin1), pin2(pin2) ,pin3(pin3), pin4(pin4) {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);
    return;
  }

  void moveForward() {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    
    digitalWrite(pin3,HIGH);
    digitalWrite(pin4,HIGH);
    return;
  }

  void moveBackward() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    
    digitalWrite(pin3,HIGH);
    digitalWrite(pin4,HIGH);
    return;
  }

  void stop() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    
    digitalWrite(pin3,HIGH);
    digitalWrite(pin4,HIGH);
    return;
  }

private:
  int pin1;
  int pin2;
  int pin3;
  int pin4;
};

MotorDriver motor1(motor1Pin1, motor1Pin2,motor1Pin3,motor1Pin4);
//MotorDriver motor2(motor2Pin1, motor2Pin2);
//MotorDriver brush(brushPin1, brushPin2);

void moveLeft() {
  Serial.println("Moving Left");
  motor1.moveForward();
  //motor2.moveBackward();
  //brush.moveForward();
  digitalWrite(pump,HIGH);
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, HIGH);
  digitalWrite(stopLED, LOW);
  return;
}

void moveRight() {
  Serial.println("Moving Right");
  motor1.moveBackward();
  //motor2.moveForward();
  //brush.moveBackward();
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, HIGH);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, LOW);
  return;
}

void stopMotors() {
  remote_btn= IR_recev();
  //digitalRead(stopButtonPin) == LOW
  if ( remote_btn==2 || digitalRead(stopButtonPin) == 0 ) {
    Serial.println("Stopping Motors");
    motor1.stop();
    //motor2.stop();
    //brush.stop();
    digitalWrite(pump,LOW);
    digitalWrite(startLED, LOW);
    digitalWrite(dryLED, LOW);
    digitalWrite(wetLED, LOW);
    digitalWrite(stopLED, HIGH);
    remote_btn = 0;
  }
  return;
}

void restartToLeft() {
  Serial.println("Restarting to Left");
  moveLeft();
  digitalWrite(startLED,HIGH);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, HIGH);
  return;
}

int IR_recev() {

  

  if (IrReceiver.decode()) {
      //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data 
      //byte remotepin=IrReceiver.decodedIRData.decodedRawData;
      switch (IrReceiver.decodedIRData.decodedRawData) {
        case 0xBA45FF00: // start button pressed which is 1 in HEX for example
        //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
          Serial.println("start1");
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
  return remote_btn;
}
void startCleaning(){
  // Read the state of the stop button
  remote_btn=IR_recev();
  //digitalRead(startButtonPin) == LOW ||
 
//    if (digitalRead(stopButtonPin) == LOW) {
//      // Stop the motors immediately
//      stopMotors();
//      return;  // Exit the function to avoid further execution
//    }

   if ( remote_btn==1 || digitalRead(startButtonPin) == 0){
    Serial.println("start2");
    // Read the states of the IR and proximity sensors
    int irlState = digitalRead(irlPin);
    int irrState = digitalRead(irrPin);
    int psr1State = digitalRead(psr1Pin);
    int psr2State = digitalRead(psr2Pin);
    int psl1State = digitalRead(psl1Pin);
    int psl2State = digitalRead(psl2Pin);

    moveRight();
    remote_btn = 0;
    // Check if conditions are met to move to the right
    if (irlState == LOW && psl1State == LOW && psl2State == LOW) {
      // Move to the right until certain conditions are met
      while (irrState == HIGH && psr1State == HIGH && psr2State == HIGH) {
        moveRight();
        // Update sensor states inside the loop
        irrState = digitalRead(irrPin);
        psr1State = digitalRead(psr1Pin);
        psr2State = digitalRead(psr2Pin);
      }
  
      // Stop the machine when conditions are met to move to the left
      stopMotors();
  
      // Check if conditions are met to move to the left
      if (irrState == LOW && psr1State == LOW && psr2State == LOW && irlState == HIGH && psl1State == HIGH && psl2State == HIGH) {
        // Move to the left until certain conditions are met
        while (irlState == HIGH && psl1State == HIGH && psl2State == HIGH) {
          moveLeft();   
          // Update sensor states inside the loop
          irlState = digitalRead(irlPin);
          psl1State = digitalRead(psl1Pin);
          psl2State = digitalRead(psl2Pin);
        }
      }
    }
  }
  return;
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


  digitalWrite(startButtonPin, HIGH);
    digitalWrite(stopButtonPin, HIGH);

}

void loop() {

  SchedBase::dispatcher();

  }
  // Check if the start button is pressed
//  if (digitalRead(startButtonPin) == LOW) {
//    // Start the program
//    startCleaning();
//  }
//
//  // Check if the stop button is pressed
//  if (digitalRead(stopButtonPin) == LOW) {
//    // Stop the machine
//    stopMotors();
//  }

  //delay(100); // Add a small delay to avoid rapid readings
