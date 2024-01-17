// IR receiver variables 

#include <IRremote.hpp>
#define IR_RECEIVE_PIN 2
int remote_btn;




//  Pins

//  Motor pins
int Motion1R_en ;
int Motion1L_en ;
int Motion1R_pwm ;
int Motion1L_pwm ;

int Motion2R_en ;
int Motion2L_en ;
int Motion2R_pwm ;
int Motion2L_pwm ;


int brushR_en ;
int brushL_en ;
int brushR_pwm ;
int brushL_pwm ;

//  Dust detector pins 
int measurePin = 0; //Connect dust sensor to Arduino A0 pin
int ledPower = 7 ;   //Connect 3 led driver pins of dust sensor to Arduino D2
int samplingTime = 280; // time required to sample signal coming out   of the sensor
int deltaTime = 40; // 
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

// Buttons
int start_btn = 6;

// LEDs

// Distance Sensors

//  Methods
void motor_forward(int r_en, int l_en,  int r_pwm, int l_pwm, int speed);
void motor_stop(int r_en, int l_en,  int r_pwm, int l_pwm, int speed);
void motor_reverse(int r_en, int l_en,  int r_pwm, int l_pwm, int speed) ;
int dust_det();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  
  //  Motors
  pinMode(Motion1R_en, OUTPUT);
  pinMode(Motion1L_en, OUTPUT);
  pinMode(Motion1R_pwm , OUTPUT);
  pinMode(Motion1L_pwm, OUTPUT);
  
  pinMode(Motion2R_en, OUTPUT);
  pinMode(Motion2L_en, OUTPUT);
  pinMode(Motion2R_pwm , OUTPUT);
  pinMode(Motion2L_pwm, OUTPUT);

  pinMode(brushR_en, OUTPUT);
  pinMode(brushL_en, OUTPUT);
  pinMode(brushR_pwm , OUTPUT);
  pinMode(brushL_pwm, OUTPUT);

  //  Dust detector 
  pinMode(ledPower,OUTPUT);

  //  Buttons
  pinMode(start_btn, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  IR_recev();
  delay(1000);
  // if the start button is pressed, or the IR receiver recieve start signal, or dust detector detects signal or the wi-fi module sent start signal 
//  if (digitalRead(start_btn) == HIGH ||  remote_btn == 1 ) {
//    motor_forward(Motion1R_en, Motion1L_en, Motion1R_pwm, Motion1L_pwm,90);
//    dust_det();
//  }

}

void motor_forward(int r_en, int l_en,  int r_pwm, int l_pwm, int speed) {
    digitalWrite(r_en, HIGH);
    digitalWrite(l_en, HIGH);
    analogWrite(r_pwm, speed);
    analogWrite(l_pwm, 0);
}

void motor_stop(int r_en, int l_en,  int r_pwm, int l_pwm, int speed) {
    digitalWrite(r_en, HIGH);
    digitalWrite(l_en, HIGH);
    analogWrite(r_pwm, 0);
    analogWrite(l_pwm, 0);
}

void motor_reverse(int r_en, int l_en,  int r_pwm, int l_pwm, int speed) {
    digitalWrite(r_en, HIGH);
    digitalWrite(l_en, HIGH);
    analogWrite(r_pwm, 0);
    analogWrite(l_pwm, speed);
}

int dust_det(){
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin); // read the dust value
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);
  // 0 - 5V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (5.0 / 1024.0);
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = 170 * calcVoltage - 0.1;
  Serial.println(dustDensity); // unit: ug/m3
  delay(1000);
  return dustDensity;
  
}

// IR Remote Receiver function 
void IR_recev() {

  remote_btn = 0;


  if (IrReceiver.decode()) {
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data 
      switch (IrReceiver.decodedIRData.decodedRawData) {
        case BA45FF00: // start button pressed which is 1 in HEX for example
        Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
        Serial.println("start");
        remote_btn = 1;
        break;

        case B946FF00: // stop button pressed which is 2 in HEX for example
        Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); 
        Serial.println("stop");
        remote_btn = 2;
        break;
      }
      IrReceiver.resume(); // Enable receiving of the next value
  }
}
