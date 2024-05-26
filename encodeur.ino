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


    //qtr
    int tst=0;
       // turn on Arduino's LED to indicate we are in calibration mode
   for (int i = 0 ; i < 200; i++)
  {
    qtr.calibrate();
    tst=1;
    delay(20);
  }
//INTERREPTEUR
    digitalWrite(launchpin, HIGH);
    current_time=millis();
int x=1;

   while(1){
   x=digitalRead(launchpin);
    // Serial.println(x);
    if(x==0){
         lunch_time=millis();
      if(lunch_time-current_time>200) break;
    }}  
  stp(500);
  int sum;
}

void loop() {
  position = qtr.readLine(s);
  current_time=millis();
  // mouleharebbi_valide_testss1();

binaire(s);
// Serial.print(s[0]);
//    Serial.print("  ");
//    Serial.print(s[1]);
//    Serial.print("  ");
//    Serial.print(s[2]);
//    Serial.print("  ");
//    Serial.print(s[3]);
//    Serial.print("  ");
//    Serial.print(s[4]);
//    Serial.print("  ");
//    Serial.print(s[5]);
//    Serial.print("  ");
//    Serial.print(s[6]);
//    Serial.print("  ");
//    Serial.print(s[7]);
//    Serial.print("  ");
//    Serial.print(s[8]);
//    Serial.print("  ");
//    Serial.print(s[9]);
//    Serial.print("  ");
//    Serial.print(s[10]);
//    Serial.print("  ");
//    Serial.print(s[11]);
//    Serial.print("  ");
//    Serial.print(s[12]);
//    Serial.print("  ");
//    Serial.print(s[13]);
//    Serial.print("  ");
//    Serial.print(s[14]);
//    Serial.print("  ");
//    Serial.print(s[15]);
//    Serial.print("  ");
//    Serial.print("position:");
//    Serial.print("  ");
//    Serial.print(position);
//    Serial.println();
int sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0];
int sumg=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8];
int sumd=s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0];
if(n==0 && sumd>7000){
  left_base=120;right_base=90;
  stp(10);
  last_time=current_time;
  n=1;
}
// if(n==1 && sumd>7000 && current_time-last_time>3000){
//   stp(10);
//   last_time=current_time;
//   n=2;
// }
if(n==1 && sum<2000 && current_time-last_time>3300){
  left_base=180;right_base=130;
  stp(100);
  reset_encR();
  reset_encL();
  while(encL_ticks<100){
  right_teli(1);
}
stp(100);  
    while(sum<=2000){
    left(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
  last_time=current_time;
  n=2;
}
//D5LA FL CHMS
if(n==2 && sum>7000 && current_time-last_time>500){
  stp(10);
  left_base=120;right_base=90;
  last_time=current_time;
  n=3;
}

if(n==3 && s[3]+s[2]+s[1]+s[0]>2500 && current_time-last_time>100){
  

  left(80);
  x++;
  if(x==19){
   
    
  reset_encR();
  reset_encL();
  while(encL_ticks<120){
  right(1);
} 
  while(sum<=2000){
    right(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  } 
    n=5;
    stp(200);}
 last_time=current_time; 
 
  }
  if(n==5 && sum>8000 && current_time-last_time>800 ){
    left_base=120;right_base=90;
    stp(100);
      reset_encR();
  reset_encL();
  while(encR_ticks<26){
  left_dd(1);
} 
  while(sum<=1000){
    left(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
  last_time=current_time;
n=6;
  
  }
  if(n==6 && sum>8000 && current_time - last_time>400 ){
    stp(5000);
          reset_encR();
  reset_encL();
  while(encR_ticks<100){
  left_dd(1);
} 
  while(sum<=2000){
    left(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
n=7;
last_time=current_time;
  }
  // chn5rju ml pause
    if(n==7 && sum>8000 && current_time - last_time>400 ){
    
          reset_encR();
  reset_encL();
  while(encR_ticks<110){
  left_dd(1);
} 
  while(sum<=2000){
    left(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
n=8;
last_time=current_time;
  }
  //demi cercle chnd5luha
    if(n==8 && sum>10000 && current_time - last_time>1300 ){
    
          reset_encR();
  reset_encL();
  while(encL_ticks<100){
  right_dd(1);
} 
  while(sum<=2000){
    right(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
n=9;
last_time=current_time;
  }
  //demi cercle chnd5rju
    if(n==9 && sumd>7000 && current_time - last_time>1000 ){
    stp(100);
          reset_encR();
  reset_encL();
  while(encL_ticks<60){
  right_dd(1);
} 
  while(sum<=2000){
    right(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
n=10;
last_time=current_time;
  }  

//B3D 5ERJA ML CERCLE
    if(n==10 && sumd>7000 && current_time - last_time>500 ){
    stp(5000);
          reset_encR();
  reset_encL();
  while(encL_ticks<60){
  right_dd(1);
} 
  while(sum<=2000){
    right(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
n=11;
last_time=current_time;
  } 
  //CENTRE MTE3 MONGELA
      if(n==11 && sumd>7000 && current_time - last_time>1000 ){
    stp(5000);
          reset_encR();
  reset_encL();
  while(encL_ticks<60){
  right_dd(1);
} 
  while(sum<=2000){
    right(1);
    binaire(s);
    position = qtr.readLine(s);
    binaire(s);
    sum=s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];

  }
  stp(100);
n=10;
last_time=current_time;
  } 
// last_time=current_time;
// n=1;
// }
// if(n==1 && sumd>7000 && current_time-last_time>800){
//   stp(100);
//   reset_encR();
//   reset_encL();
//   while(encR_ticks<45){
//   left(1);
// }
// stp(100);
// while(s[7]+s[6]+s[8]+s[9]>1000){
//   qtr.readLine(s);
//   binaire(s);
//   left_teli(1);
// }
// stp(200);
// reset_encR();
//   reset_encL();
// while(encL_ticks<1400*0.7 && encR_ticks<103*0.7){
//   forwardslow(1);
// }
// stp(200);
// while(s[7]+s[10]+s[9]+s[8]>1000){
//   qtr.readLine(s);
//   binaire(s);
//   right_dd(1);
// }
// stp(200);
// while(s[7]+s[6]+s[8]+s[9]>1000){
//   qtr.readLine(s);
//   binaire(s);
//   right_teli(1);
// }
// stp(20000);
// last_time=current_time;
// n==2;
// }

// if(n==2 && sumd<2000 && current_time-last_time>400){
//   stp(12556);
// }






  PID();
forwardPID();


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




//decalaration des fonctions
void PID(){
  binaire(s);
  position = qtr.readLine(s);
  binaire(s);
  error=position-6500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  left_speed=left_base+PIDvalue;
  right_speed=right_base-PIDvalue;
  left_speed=min(180,max(left_speed,0));
  right_speed=min(180,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PIDW(){
  position = qtr.readLineM(s);
  error=position-3500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));}


void pidforcee(){
  position = qtr.readLine(s);
  if(position<2000){left(1);}
  else if (position>13000){right(1);} 
 else{ error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0)); } 
}
void PID_noir(){
  position = qtr.readLine(s,QTR_EMITTERS_ON,1,true);
  error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
   right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(180,max(left_speed,0));
  right_speed=min(180,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}

void PID_noir_special(){
  position = qtr.readLine(s,QTR_EMITTERS_ON,1,false);
  error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  left_speed=left_base-PIDvalue;
  right_speed=right_base+PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}

void PID_special(){
  bool v=false;
  position = qtr.readLine(s,QTR_EMITTERS_ON,0,v);
  error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void forwardPID(){
  
  analogWrite(rightF,right_speed);
   analogWrite(leftF,left_speed);
  analogWrite(rightR,0);
  analogWrite(leftR,0);

}
void forwardslow(int x){
  
analogWrite(rightF,80);
   analogWrite(leftF,100);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void forwardfast(int x){
  
analogWrite(rightF,200);
   analogWrite(leftF,250);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void forwardM(int x){
  
analogWrite(rightF,150);
   analogWrite(leftF,150);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void forwardmedium(int x){
  
analogWrite(rightF,120);
   analogWrite(leftF,120);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void forwardG(int l, int r, int x){
  
analogWrite(rightF,r);
   analogWrite(leftF,l);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}


void forwardf(int x){
  
analogWrite(rightF,255);
   analogWrite(leftF,255);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void right(int x){
  
  analogWrite(rightF,120);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void right_sep(int x){
  
  analogWrite(rightF,150);
   analogWrite(leftF,60);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}

void right_dd(int x){
  
  analogWrite(rightF,120);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,120);
  delay(x);
}
void right_dd1(int x){
  
  analogWrite(rightF,180);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,180);
  delay(x);
}
void right_dd2(int x){
  
  analogWrite(rightF,200);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,200);
  delay(x);
}
void left_dd(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,120);
  analogWrite(rightR,120);
  analogWrite(leftR,0);
  delay(x);
}
void left_dd1(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,180);
  analogWrite(rightR,180);
  analogWrite(leftR,0);
  delay(x);
}
void left(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,120);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void stp(long int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void back(int x){
   analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,100);
  analogWrite(leftR,100);
  delay(x);
}
void stp1(){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
}
void leftmaze(int x){
  analogWrite(rightF,30);
   analogWrite(leftF,60);//80
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void rightmaze(int x){
  analogWrite(rightF,100);
   analogWrite(leftF,40);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void rightmaze_1(int x){
  analogWrite(rightF,90);
   analogWrite(leftF,32);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void speed(){
  int i=158; int j=120;
  while(i<255){
    left_speed=i;
    right_speed=j;
    i+=0.1;j+=0.1;
  }
}
void dhaw(int x) {
  digitalWrite(53, HIGH);
  digitalWrite(red, HIGH);
  delay(50);
  digitalWrite(53, LOW);
  digitalWrite(red, LOW);
}
void dhawfinale(int x) {
  digitalWrite(green, HIGH);
  digitalWrite(red, HIGH);
  delay(50);
  digitalWrite(green, LOW);
  digitalWrite(red, HIGH);
  delay(70);
}

void mouleharebbi_valide_N(){
  if (s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]>=14000){
  forwardM(1);
  }


else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+2000<s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]) {
  right_dd1(1);
} 
else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]>2000+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]) {
  left_dd1(1);
}
else {
  forwardM(1);
}
}
void mouleharebbi_valide(){
  if (s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]>=14000){
  forwardM(1);
  }


else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+2000<s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]) {
  left_dd1(1);
} 
else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]>2000+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]) {
  right_dd1(1);
}
else {
  forwardM(1);
}
}

void mouleharebbi_valide_favors_turn_right(){
if(s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]<=1000&&s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]>=3500) {
  right_dd1(1);

} 
else if(s[5]+s[4]+s[3]+s[2]+s[1]+s[0]>=3500&&s[15]+s[14]+s[13]+s[12]+s[11]+s[10]>=3500) {
  left_dd1(1);

} 

else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+2000<s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]) {
  left_dd1(1);
} 
 
else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]<=2000) {
  left_dd(1);

} 
else {
  forwardM(1);
}
}



void mouleharebbi_valide_testss(){
  if (s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]>=14000){
  forward_mrt(1);
  }


else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]+2000<s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]) {
  left_dd(1);
} 
else if(s[15]+s[14]+s[13]+s[12]+s[11]+s[10]+s[9]+s[8]>2000+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]) {
  right_dd(1);
}
else {
  forward_mrt(1);
}
}


void forward_mrt(int x){
 //80 
analogWrite(rightF,80);
   analogWrite(leftF,80);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void mouleharebbi_valide_testss1(){
int sum=s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]  ;
  if (s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1]+s[0]>=6000){
  forwardslow(1);
  }


else if(s[0]+s[1]+s[2]+s[3]+1000<s[5]+s[6]+s[7]+s[4]) {
  while(sum<2000){
    left(1);
    position = qtr.readLine(s);
    sum=s[8]+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];      
    } 
} 
else if(s[0]+s[1]+s[2]+s[3]>s[5]+s[6]+s[7]+s[4]+1000) {
   while(sum<2000){
    right(1);
    position = qtr.readLine(s);
    sum=+s[7]+s[6]+s[5]+s[4]+s[3]+s[2]+s[1];      
    }  
  
}
else {
  forwardslow(1);
}
}
void binaire(int s[16]){
  int i=0;
while(i<16){
  if (s[i]<500){
s[i]=0;  }
else{
  s[i]=1000;
}
i++;
}
}
void right_teli(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,120);
  analogWrite(leftR,0);
  delay(x);
}
void left_teli(int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,120);
  delay(x);
}
