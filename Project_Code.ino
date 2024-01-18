const int startButtonPin = 2;  // Push button for starting the program
const int stopButtonPin = 3;   // Push button for stopping the machine

const int motor1Pin1 = 4;  // Motor1 input 1 pin
const int motor1Pin2 = 5;  // Motor1 input 2 pin
const int motor2Pin1 = 6;  // Motor2 input 1 pin
const int motor2Pin2 = 7;  // Motor2 input 2 pin

const int irlPin = 8;          // IR sensor left
const int irrPin = 9;          // IR sensor right
const int psr1Pin = 10;        // Proximity sensor right 1
const int psr2Pin = 11;        // Proximity sensor right 2
const int psl1Pin = 12;        // Proximity sensor left 1
const int psl2Pin = 13;        // Proximity sensor left 2

const int startLED = A0;  // LED for start
const int dryLED = A1;    // LED for moving to the right
const int wetLED = A2;    // LED for moving to the left
const int stopLED = A3;   // LED for stopping

class MotorDriver {
public:
  MotorDriver(int pin1, int pin2) : pin1(pin1), pin2(pin2) {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }

  void moveForward() {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }

  void moveBackward() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }

  void stop() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }

private:
  int pin1;
  int pin2;
};

MotorDriver motor1(motor1Pin1, motor1Pin2);
MotorDriver motor2(motor2Pin1, motor2Pin2);

void moveLeft() {
  Serial.println("Moving Left");
  motor1.moveForward();
  motor2.moveBackward();
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, HIGH);
  digitalWrite(stopLED, LOW);
}

void moveRight() {
  Serial.println("Moving Right");
  motor1.moveBackward();
  motor2.moveForward();
  digitalWrite(startLED, HIGH);
  digitalWrite(dryLED, HIGH);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, LOW);
}

void stopMotors() {
  Serial.println("Stopping Motors");
  motor1.stop();
  motor2.stop();
  digitalWrite(startLED, LOW);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, HIGH);
}

void restartToLeft() {
  Serial.println("Restarting to Left");
  moveLeft();
  digitalWrite(startLED,HIGH);
  digitalWrite(dryLED, LOW);
  digitalWrite(wetLED, LOW);
  digitalWrite(stopLED, HIGH);
}


void startCleaning(){
  // Read the state of the stop button
  if (digitalRead(stopButtonPin) == LOW) {
    // Stop the motors immediately
    stopMotors();
    return;  // Exit the function to avoid further execution
  }

  // Read the states of the IR and proximity sensors
  int irlState = digitalRead(irlPin);
  int irrState = digitalRead(irrPin);
  int psr1State = digitalRead(psr1Pin);
  int psr2State = digitalRead(psr2Pin);
  int psl1State = digitalRead(psl1Pin);
  int psl2State = digitalRead(psl2Pin);

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

void setup() {
  Serial.begin(9600);
  pinMode(startButtonPin, INPUT);
  pinMode(stopButtonPin, INPUT);
  pinMode(startLED, OUTPUT);
  pinMode(dryLED, OUTPUT);
  pinMode(wetLED, OUTPUT);
  pinMode(stopLED, OUTPUT);
}

void loop() {
  // Check if the start button is pressed
  if (digitalRead(startButtonPin) == LOW) {
    // Start the program
    startCleaning();
  }

  // Check if the stop button is pressed
  if (digitalRead(stopButtonPin) == LOW) {
    // Stop the machine
    stopMotors();
  }

  delay(100); // Add a small delay to avoid rapid readings
}
