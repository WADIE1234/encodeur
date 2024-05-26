

int launchpin=7;

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




