#include <NewPing.h>
#include<Servo.h>

#define MAX_DISTANCE 200

//Motor sheld Pins
//M1
int enA = 5;
int in1 = 3;
int in2 = 4;
//M2
int enB = 6;
int in3 = 7;
int in4 = 8;

int setp=0;

int encoderValue=0; 
void count(void); 
///////////////////////////////////////ABOUT SERVOS////////////////////////////////////////
// D//eclare the Servo pin
int servoPin =0;
// Create a servo object
Servo ServoA;
Servo ServoB;

////////////END ABOUT SERVOS////////////

///////////////////ABOUT COLOR SENSOR///////////////////////////
const int s0 = 40;  
const int s1 = 41;  
const int s2 = 42;  
const int s3 = 43;  
const int out = 13;   
// LED pins connected to Arduino
//int redLed = 2;  
//int greenLed = 3;  
//int blueLed = 4;
// Variables  
int red = 0;  
int green = 0;  
int blue = 0;  

int redflag = 0;  
int greenflag = 0;  
int blueflag = 0; 
      boolean getColour=false;

/////////////////////////////////////////////////////// Ultra Sonics///////////////////
int trigLF = 18;    //Trig - green Jumper
int echoLF = 19;    //Echo - yellow Jumper

int trigRF = 14;    //Trig - green Jumper
int echoRF = 15;    //Echo - yellow Jumper

int trigLB = 10;    //Trig - green Jumper
int echoLB = 11;    //Echo - yellow Jumper

int trigRB = A10;    //Trig - green Jumper
int echoRB = A11;    //Echo - yellow Jumper

int trigF = 16;    //Trig - green Jumper
int echoF = 17;    //Echo - yellow Jumper
/////////////////////////////////////////////////////// Ultra Sonics END////////////////
long duration, cmF,cmLB,cmLF,cmRB,cmRF;

NewPing leftfront(trigLF, echoLF, MAX_DISTANCE);
NewPing front(trigF, echoF, MAX_DISTANCE);
NewPing rightfront(trigRF, echoRF, MAX_DISTANCE);
NewPing leftback(trigLB, echoLB, MAX_DISTANCE);
NewPing rightback(trigRB, echoRB, MAX_DISTANCE);

int beepPin=13;
void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);
delay(2000);

pinMode(21,INPUT);
  delay(300);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(beepPin, OUTPUT);
attachInterrupt(2,count,FALLING);//A
  pinMode(trigLF, OUTPUT);
  pinMode(echoLF, INPUT);
  
  pinMode(trigRF, OUTPUT);
  pinMode(echoRF, INPUT);
  
  pinMode(trigLB, OUTPUT);
  pinMode(echoLB, INPUT);
  
  pinMode(trigRB, OUTPUT);
  pinMode(echoRB, INPUT);
  
  pinMode(trigF, OUTPUT);
  pinMode(echoF, INPUT);

  digitalWrite(trigF, LOW);
  
  delayMicroseconds(5);

  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);  
  pinMode(out, INPUT);  
  //pinMode(redLed, OUTPUT);  
  //pinMode(greenLed, OUTPUT);  
 // pinMode(blueLed, OUTPUT);  
  digitalWrite(s0, HIGH);  
  digitalWrite(s1, HIGH); 


////////////ABOUT SERVOS////////////
   ServoA.attach(9);//right
ServoB.attach(12);//left

ServoB.write(90);
ServoA.write(90); //  Start close

////////////END ABOUT SERVOS////////////
}

int mode = 0;
//0-COLOR DETECT
//1-MAZE SOLVE
//2-COLOR LINE FOLLOW
//3-GRAP THE LOAD
void loopx() {
forward(70,70,1,1);//forward(int r,int l,int rd,int ld)
/*if ((cmRF >6 && cmRF <9) && (cmRB >6 && cmRB <9))
       {
        resetposition(0,0,1,1);
        delay(500);
        setp=1;
        }
        if(cmRF >= 9 && cmRB <= 6)
        {
          Serial.print("-turn slow right rest");
          resetposition(60,0,1,1);
          }
         if(cmRF <=6 && cmRB >=9) 
         {
          Serial.print("-turn slow left rest");
          resetposition(0,60,1,1);
          }*/
}

void loop() {
  // put your main code here, to run repeatedly:
cmLF = getDistance(trigLF,echoLF);
cmLB = getDistance(trigLB ,echoLB);
cmRF = getDistance(trigRF,echoRF);
cmRB = getDistance(trigRB,echoRB);
cmF = getDistance(trigF,echoF);

cmLF = leftfront.ping_cm();
cmF = front.ping_cm();
cmRF = rightfront.ping_cm();
cmLB = leftback.ping_cm();
cmRB = rightback.ping_cm();

//Algorithem..
//left
//forward
//right
Serial.print("  F :"); 
Serial.print(cmF); 
Serial.print("  LF :"); 
Serial.print(cmLF); 
Serial.print("  LB :"); 
Serial.print(cmLB); 
Serial.print("  RF :"); 
Serial.print(cmRF); 
Serial.print("  RB :"); 
Serial.println(cmRB); 

    if(cmF >12)
      {
     if(cmRF >9 && cmRF <12)
      {
        Serial.print("-forward");
        forward(80,80,1,1);//forward(int r,int l,int rd,int ld)
        }
    if(cmRF >=9)//RF
       {
        Serial.print("-turn slow right");
        forward(75,80,1,1);//forward(int r,int l,int rd,int ld)
        } 
    if(cmRF <=9)//RF
       {
        Serial.print("-turn slow left");
        forward(80,75,1,1);//forward(int r,int l,int rd,int ld)
        }   
    }
    if (cmF <=12)
    {
      delay(60);
      stop(500);
      if(cmF <=3)
      {
        backward(60,60,-1,-1);//forward(int r,int l,int rd,int ld)
        }
      
      delay(60);
      //stop(500);
      //resetposition(); ///////////////////////////
      
      if(cmLF <=20 && cmRF >30 && cmF !=0 && cmF !=1149){Serial.print("right 90"); forward(90,90,-1,1);}

      if(cmLF >30 && cmRF >30 && cmF !=0 && cmF !=1149){Serial.print("right 90 T"); forward(90,90,-1,1);}

      if(cmRF <=20 && cmLF >30 && cmF !=0 && cmF !=1149){Serial.print("left 90"); forward(90,90,1,-1);}

      if(cmRF <=15 && cmLF <= 15 && cmF !=0 && cmF !=1149){Serial.print("180"); forward(90,90,-1,1);}
    }
      colordetect();
}


long getDistance(int tp,int ep) {
  digitalWrite(tp, HIGH);
  delayMicroseconds(10);
  digitalWrite(tp, LOW);
  duration = pulseIn(ep, HIGH, 100000);
  return (duration / 2) / 29.1;
}

void mpower(int motor, int rotation, int spd) {
  int pwm;
  int pA;
  int pB;
  if (motor == 1) {
    pwm = enA;
    pA = in1;
    pB = in2;
  } else if (motor == 2) {
    pwm = enB;
    pA = in3;
    pB = in4;
  } else {
    return;
  }
  if (rotation == 0) {
    digitalWrite(pA, LOW);
    digitalWrite(pB, LOW);
  } else if (rotation == 1) {
    digitalWrite(pA, HIGH);
    digitalWrite(pB, LOW);
  } else if (rotation == -1) {
    digitalWrite(pA, LOW);
    digitalWrite(pB, HIGH);
  }
  analogWrite(pwm, spd);

}
void stop(int d) {
  mpower(1, 0, 100);
  mpower(2, 0, 100);
  delay(d);
}

/*void stop(int l,int r,int ld,int rd){
  mpower(1,0,100);
        mpower(2,0,100);
      delay(100);
  }*/
  
  void encoderWrite(int val,int d){//left d 1 righr d -1
    encoderValue=0;
    Serial.print("Encoder Value="); 
Serial.println(encoderValue); 
while(val>encoderValue){
mpower(1,d,100);
mpower(2,-d,100);
}
 // mpower(1,-1,100);
//mpower(2,1,100);
//delay(50);
  mpower(1,0,100);
mpower(2,0,100);
  
  
    }

    void forward(int l,int r,int ld,int rd){
      
       mpower(1,rd,r);
        mpower(2,ld,l);
      }
  void backward(int l,int r,int ld,int rd){
      
       mpower(1,rd,r);
        mpower(2,ld,l);
      }    
void resetposition()
{
  while(setp==0)
   {
    cmF = front.ping_cm();
    cmLF = leftfront.ping_cm();
    cmRF = rightfront.ping_cm();
    cmRB = rightback.ping_cm();
    cmLB = leftback.ping_cm();
    
    if ((cmRF >9 && cmRF <12) && (cmLF >9 && cmLF <12))
       {
        forward(0,0,1,1);
        delay(500);
        setp=1;
        }
        if(cmRF >= 9 && cmLF <= 6)
        {
          Serial.print("-turn slow right rest");
          forward(60,0,1,1);
          }
         if(cmRF <=6 && cmLF >=9) 
         {
          Serial.print("-turn slow left rest");
          forward(0,60,1,1);
          }
    }
  }
   
      void count(){
encoderValue++;
}
void beep(int d){
  digitalWrite(beepPin,HIGH);
  delay(d);
  digitalWrite(beepPin,LOW);
  }

void leftbackCheck(){
  cmLF = getDistance(trigLF,echoLF);
cmLB = getDistance(trigLB ,echoLB);
cmRF = getDistance(trigRF,echoRF);
cmRB = getDistance(trigRB,echoRB);
cmF = getDistance(trigF,echoF);

    if(cmLB >=9)//RF
       {
        Serial.print("-turn slow left");
        forward(75,80,1,1);
        } 
    else if(cmRB <=9)//RF
       {
        Serial.print("-turn slow right");
        forward(80,75,1,1);
        }  
} 
void colordetect()
{
if(redflag<6 || greenflag<6 || blueflag<6){
  color(); 
  Serial.print("R Intensity:");  
  Serial.print(red, DEC);  
  Serial.print(" G Intensity: ");  
  Serial.print(green, DEC);  
  Serial.print(" B Intensity : ");  
  Serial.print(blue, DEC);  
}
  if (red < blue && red < green && red < 202)//&& red>7
  {  
   Serial.println(" - (Red Color)");  
  
   redflag++;
  }  

  else if (blue < red && blue < green)   
  {  
   Serial.println(" - (Blue Color)");  
      Serial.println(blueflag);  
  // digitalWrite(redLed, LOW);  
  // digitalWrite(greenLed, LOW);  
  // digitalWrite(blueLed, HIGH); // Turn BLUE LED ON  
   blueflag++;
  }  


  else if (green < red && green < blue)  
  {  
   Serial.println(" - (Green Color)");  
  // digitalWrite(redLed, LOW);  
  // digitalWrite(greenLed, HIGH); // Turn GREEN LED ON 
  // digitalWrite(blueLed, LOW);  
   greenflag++;
  }  
  else{
  Serial.println();  
  }
  delay(300);   
 // digitalWrite(redLed, LOW);  
 // digitalWrite(greenLed, LOW);  
 // digitalWrite(blueLed, LOW);  

if(redflag==5 || blueflag==5 || greenflag==5 ){
  
  //////////////////////////////////////////////////////////////load codings///////////////////////////////

  // Make servo go to 0 degrees
ServoB.write(115);
ServoA.write(90);//open
delay(1000);

delay(1000);
ServoB.write(85);
ServoA.write(120); //  Start close
delay(1000);
//delay(1000);
//Servo2.write(115);
//Servo1.write(90);//open


delay(1000);
//Servo3.write(90);
 //motor2.run(FORWARD);
//delay(5000);
 //motor2.run(RELEASE);
//delay(2000);
ServoB.write(85);
ServoA.write(120);  //close
// Make servo go to 90 degrees
delay(500);

// Make servo go to 0 degrees


// Make servo go to 90 degrees

delay(1000);
  
//  getColour=true;
redflag = 6;  
greenflag = 6;  
blueflag = 6; 
  //////////////////////////////////////////////////////////////END load codings///////////////////////////////
  }
}
  void color()  
{    
  digitalWrite(s2, LOW);  
  digitalWrite(s3, LOW);  
  //count OUT, pRed, RED  
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s3, HIGH);  
  //count OUT, pBLUE, BLUE  
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s2, HIGH);  
  //count OUT, pGreen, GREEN  
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  
}
 
  
