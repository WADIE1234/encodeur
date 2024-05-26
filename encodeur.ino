#include <QTRSensors.h>
int led=5;
int red=49;
int green=53;

int launchpin=7;
const uint8_t sensorCount=16;
// uint16_t s[SensorCount];
int s[16];
QTRSensorsRC qtr((unsigned char[]){A0, A1, A2, A3, A4, A5, A6,A7,A8, A9, A10, A11, A12, A13, A14,A15}, 16);

int j=10000000000;
//hetha kl 3eda
int position;
int n=0;
int c=0;
//PID
int right_speed,left_speed; 
float last_time2;
int x=0;
float lt;

// float kp=0.026,kd=0.05,ki=0,P,D,I;

// float kp=0.022,kd=0.08,ki=0,P,D,I; 110?150
 float kp=0.025,kd=0.07,ki=0,P,D,I; 
float PIDvalue,lasterror,error,error1,lasterror1;
int left_base=180,right_base=130;
//motors
int rightF=6;
int rightR=5;
int leftF=8;
int leftR=9;
unsigned int current_time,last_time,lunch_time;

//for encodeur
int ENCODERAL=20 ;
int ENCODERAR=19 ;

volatile unsigned int encR_ticks = 0;
volatile unsigned int encL_ticks = 0;
// Function to increment right encoder tick count (Do not use directly)
void read_EncoderAR()
{
    encR_ticks++;
}
unsigned int get_encoderR(){
  return encR_ticks;
}
void reset_encR(){
  encR_ticks=0;
}
void read_EncoderAL()
{
    encL_ticks++;
}
unsigned int get_encoderL(){
  return encL_ticks;
}
void reset_encL(){
  encL_ticks=0;
}
void setup() {
  Serial.begin(9600);
  //MOTEURS
  pinMode(rightF,OUTPUT);
  pinMode(leftF,OUTPUT);
  pinMode(rightR,OUTPUT);
  pinMode(leftR,OUTPUT); 
  // put your setup code here, to run once:
pinMode(ENCODERAL, INPUT_PULLUP);
pinMode(ENCODERAR, INPUT_PULLUP);
// Attach interrupts for both encoders on rising edges
    attachInterrupt(digitalPinToInterrupt(ENCODERAL), read_EncoderAR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODERAR), read_EncoderAL, RISING);


  

}

void loop() {
 

  // put your main code here, to run repeatedly:
// Serial.print(encR_ticks);
// Serial.print("++");
// Serial.print(encL_ticks);
// Serial.println("");
// forward(500);
// stp(3000);
// while(encL_ticks<1400*1.25 && encR_ticks<103*2){
//   forward(1);
// }
// while(encL_ticks<125){
//   right(1);
// }

}




