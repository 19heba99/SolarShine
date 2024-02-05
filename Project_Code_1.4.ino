// Tasks schedule
#include <ExampleConstants.h>
#include <SchedBase.h>
#include <SchedTask.h>
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 2
int remote_btn;

// dust sensor
const int dustPin = A12;

// Buttons
const int startButtonPin = 3;  // Push button for starting the program
const int stopButtonPin = 4;   // Push button for stopping the machine

// Motors
const int motor1Pin1 = 5;  // Motor1 input 1 pin (motion)
const int motor1Pin2 = 6;  // Motor1 input 2 pin (motion)
const int motor2Pin1 = 7;  // Motor2 input 1 pin (motion)
const int motor2Pin2 = 8;  // Motor2 input 2 pin (motion)
const int brushPin1 = 9;   // Brush input 1 pin
const int brushPin2 = 10;  // Brush input 2 pin

// Sensors
const int psr1Pin = 13;   // Proximity sensor right 1
const int psl1Pin = 15;   // Proximity sensor left 1
const int en_S = 17;

int PXR_state;
int PXL_state;

// LEDs
const int startLED = A0;  // LED for start
const int dryLED = A1;    // LED for moving to the right
const int wetLED = A2;    // LED for moving to the left
const int stopLED = A3;   // LED for stopping

// Pump
const int pump = A4; // Pump relay

// Methods schedule
int IR_recev(); // Remote control
void cleaningProcess();
void startCleaning();
void stopMotors();
int dustSensor(int dustPin);

SchedTask startC(0, 100, startCleaning);      // Define the start cleaning (dispatch now, every 1 sec)
SchedTask stopCleaning(0, 500, stopMotors); // Define the stop cleaning (dispatch after 1 sec, every 1 sec)
//SchedTask restartC(2000, 1000, restartCleaning);

class MotorDriver {
public:
  MotorDriver(int pin1, int pin2, int pin3) : pin1(pin1), pin2(pin2), pin3(pin3)
  {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);
  }

  void moveForward()
  {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, HIGH);
    return;
  }

  void moveBackward()
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    digitalWrite(pin3, HIGH);
    return;
  }

  void Break()
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, HIGH);
    return;
  }

private:
  int pin1;
  int pin2;
  int pin3;
};

MotorDriver motor1(motor1Pin1, motor1Pin2, en_S);
MotorDriver motor2(motor2Pin1, motor2Pin2, en_S);
MotorDriver brush(brushPin1, brushPin2, en_S);

void moveLeft()
{
  Serial.println("Moving Left");
  motor1.moveForward();
  motor2.moveBackward();
  brush.moveForward();
  digitalWrite(pump, HIGH);
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, HIGH);
  digitalWrite(stopLED, LOW);
  return;
}

void Restart()
{
  Serial.println("reposition");
  motor1.moveForward();
  motor2.moveBackward();
  brush.moveForward();
  digitalWrite(pump, LOW);
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, HIGH);
  return;
}

void moveRight()
{
  Serial.println("Moving Right");
  motor1.moveBackward();
  motor2.moveForward();
  brush.moveBackward();
  digitalWrite(pump, LOW);
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, HIGH);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, LOW);
  return;
}

void stopMotors()
{
  // do we need an if statment ??
  int remote_btn= IR_recev();
  if ( remote_btn == 2)
  {
    Serial.println("Stopping Motors");
    motor1.Break();
    motor2.Break();
    brush.Break();
    digitalWrite(pump, LOW);
    digitalWrite(startLED, LOW);
    digitalWrite(dryLED, LOW);
    digitalWrite(wetLED, LOW);
    digitalWrite(stopLED, HIGH);
  }
  
  return;
}

int IR_recev()
{
  int remote_btn =0;
  // ? delay(10);
  if (IrReceiver.decode())
  {
    switch (IrReceiver.decodedIRData.decodedRawData)
    {
    case 0xBA45FF00: // start button pressed which is 1 in HEX for example
      Serial.println("start");
      remote_btn = 1;
      delay(50);
      break;

    case 0xB946FF00: // stop button pressed which is 2 in HEX for example
      Serial.println("stop");
      remote_btn = 2;
      delay(50);
      break;
    }
    IrReceiver.resume(); // Enable receiving of the next value
  }
  return remote_btn;
}

void cleaningProcess(bool flag){
  // Move to the right until until the end -- dry 
      while (PXR_state == HIGH && !(flag))
      {
        moveRight();
        // Update sensor states inside the loop
        PXR_state = digitalRead(psr1Pin);
        PXL_state = digitalRead(psl1Pin);
        int remote_btn= IR_recev();
        if ( remote_btn == 2)
        {
          stopMotors()
          flag=true;
          break;
        }
        delay(100);
      }
       // Move to the left until the startin position -- wet 
      while(PXL_state == HIGH && !(flag))
      {
        moveLeft();
        // Update sensor states inside the loop
        PXR_state = digitalRead(psr1Pin);
        PXL_state = digitalRead(psl1Pin);
        int remote_btn= IR_recev();
        if ( remote_btn == 2)
        {
          stopMotors()
          flag=true;
          break;
        }
        delay(100);
      }
      return;
}

void startCleaning()
{
  bool flag =false;
  
  //int dustSignal = dustSensor(dustPin);

  // Check if the conditions are met to start 
  PXR_state = digitalRead(psr1Pin);
  PXL_state = digitalRead(psl1Pin);
  
  if(IR_recev() == 1 )
  {
    
    //  First case with the robot at the starting  point
      if (PXL_state == LOW && PXR_state == HIGH)
    {
      cleaningProcess(flag);
    }
    
    // Second case with the robot at the ending point
    // Check if conditions are met to move to the left
    else if (PXL_state == HIGH && PXR_state == LOW)
    {
      // Move to the left until starting postion
      while (PXL_state == HIGH)
      {
        Restart() // Restart the robot until the starting position
        // Update sensor states inside the loop
        PXR_state = digitalRead(psr1Pin);
        PXL_state = digitalRead(psl1Pin);
        int remote_btn= IR_recev();
        if ( remote_btn == 2)
        {
          stopMotors()
          flag=true;
          break;
        }
        delay(100);
      }
      // Start over
      cleaningProcess(flag);
    }
    
    // Third case with the robot at the center
    else if (PXL_state == HIGH && PXR_state == HIGH)
    {
      // Move to the left until the starting position 
      while (PXL_state == HIGH)
      {
        Restart(); // Restart the robot until the starting position
        // Update sensor states inside the loop
        PXR_state = digitalRead(psr1Pin);
        PXL_state = digitalRead(psl1Pin);
        int remote_btn= IR_recev();
        if ( remote_btn == 2)
        {
          stopMotors()
          flag=true;
          break;
        }
        delay(100);
      }
      // Start over
      cleaningProcess(flag);
    }
    
    else
    {
      Serial.print("Error All Sensors are High");
    }
    
    stopMotors()
    delay(100);
    // ? break;
  }
  return;
}

int dustSensor(int dustPin)
{

  int dustRatio = analogRead(dustPin);

  if (dustRatio >= 20)
  {
    while (dustRatio >= 20)
    {
    }
    return 1;
  }
  return 0;
}

void setup()
{
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  pinMode(startButtonPin, INPUT);
  pinMode(stopButtonPin, INPUT);
  pinMode(psr1Pin, INPUT);
  pinMode(psl1Pin, INPUT);
  pinMode(en_S, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(brushPin1, OUTPUT);
  pinMode(brushPin2, OUTPUT);
  pinMode(pump, OUTPUT);
  pinMode(startLED, OUTPUT);
  pinMode(dryLED, OUTPUT);
  pinMode(wetLED, OUTPUT);
  pinMode(stopLED, OUTPUT);
}

void loop()
{
  SchedBase::dispatcher();
}
