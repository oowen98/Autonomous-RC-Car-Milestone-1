
#include <Servo.h>
#include <Wire.h>

Servo ESC;
Servo Savox_Servo;
//Servo Test;

//External Interrupts
volatile int ch3_pwm_value = 0;
volatile int ch3_prev_time = 0;

//SDA = A4. SCL = A5
int i2c_Address_Arduino = 0x40; //Arduino i2c Address
unsigned int data[10] = {0}; //Data coming from the Jetson
unsigned int Throttle_i2c; //Throttle Value coming from Jetson over i2c
unsigned int Steering_i2c; //Steering Value coming from Jetson over i2c

int newdata_flag = 0;

//Transmitter Values
int ch1_steering = 90;
int ch2_throttle = 100;
int ch3_val = 0;

//Output Values to ESC and Servo
int Steering_PWM = 90;
int Throttle_PWM = 100;
int Throttle_PWM_prev = 100;
int Throttle_neutral = 100;

//Pins
int ESC_pin = 6; //Outputs to the ESC and Servo Signal Wire
int Servo_pin = 9;

int ch1_pin = 8; //Inputs from the Receiver
int ch2_pin = 5;
int ch3_pin = 12; 

int mux_select = 10; //Output to the multiplexer if using

int current_Time = 0;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(0, rising, RISING); //Using interrupts to determine ch3 PWM instead of PulseIn function
  //Output pins to the ESC and Servo
  ESC.attach(ESC_pin, 1000, 2000); //Best to use oscilloscope to see TX/RX PWM Width 
  Savox_Servo.attach(Servo_pin, 1000, 2000);

  //Pin Setups
  pinMode(ch1_pin, INPUT);
  pinMode(ch2_pin, INPUT);
  pinMode(ch3_pin, INPUT);
  
  pinMode(mux_select, OUTPUT);

  Serial.begin(9600);
  while(!Serial){} //Wait for Serial to begin before continuing

  //i2c Setup
  Wire.begin(i2c_Address_Arduino);
  Wire.onReceive(receiveData); //When Jetson sends data, call receiveData
  Wire.onRequest(sendData);
}

void rising(){
  attachInterrupt(0,falling, FALLING);
  ch3_prev_time = micros();
}

void falling(){
  attachInterrupt(0,rising,RISING);
  ch3_pwm_value = micros()-ch3_prev_time;
  //Serial.print(ch3_pwm_value);
}

void receiveData(int Bytes){
  
  int num_Bytes = Wire.available();
  //Serial.print("Length: ");
  //Serial.print(num_Bytes);
  if(num_Bytes == 0 || num_Bytes > 4){
    Serial.print("Error");
    Serial.println(num_Bytes);
    return;
  }
  for (int i = 0; i < num_Bytes; i++){
    data[i] = Wire.read();
  }

  //Jetson will send Throttle and Steering in the form
  //[Throttle, Steering] (data[2], data[3]) 
  Throttle_i2c = data[2];
  Steering_i2c = data[3];
  Serial.print(Throttle_i2c);
  Serial.print("   ");
  Serial.println(Steering_i2c);

  newdata_flag = 1;
  //Savox_Servo.write(127);
  //ESC.write(Throttle_i2c);
}

void sendData(){
  Wire.write(Throttle_PWM);
}

void loop() {
  // put your main code here, to run repeatedly:
  //ch3_val = pulseIn(ch3_pin, HIGH); //Value of ch3 Switch on Transmittter
  //Serial.print("Ch3 Val: ");
  //Serial.println(ch3_val);
  
  //Transmitter Control, determine values from testing ch3 Range
  if((ch3_pwm_value > 1700) && (ch3_pwm_value < 2400)){ 
    digitalWrite(mux_select, HIGH); 
  }
  
  //Else, Jetson Control
  else {//if ((ch3_pwm_value > 700) && (ch3_pwm_value < 1200)){
    digitalWrite(mux_select, LOW);
    if (newdata_flag == 1){
      Savox_Servo.write(Steering_i2c);
      ESC.write(Throttle_i2c);
      newdata_flag = 0;
    }
   
  }
    
  //previous_Time = current_Time;
  //current_Time = millis();
  //elapsed_Time = (current_Time - previous_Time) / 1000;
  
  //SerialPrint();
 
}


void SerialPrint(){
  Serial.print(current_Time / 1000);
  Serial.print(",");
  
  Serial.print("Steering: (i2c, ch1, PWM)");
  Serial.print(",");
  Serial.print(Steering_i2c);
  Serial.print(",");
  Serial.print(ch1_steering);
  Serial.print(",");
  Serial.print(Steering_PWM);
  Serial.print(",");

  Serial.print("Throttle: (i2c, ch2, PWM)");
  Serial.print(",");
  Serial.print(Throttle_i2c);
  Serial.print(",");
  Serial.print(ch2_throttle);
  Serial.print(",");
  Serial.print(Throttle_PWM);
  Serial.print(",");

}
