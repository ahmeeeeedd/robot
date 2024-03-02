# define irSensorMiddle A3
# define irSensorRight A2
# define irSensorLeft A4
# define irSensorERight A1
# define irSensorELeft A5
# define USFT 7
# define USFE 8
# define USRT 12
# define USRE 13
#define IN1 1   
#define IN2 2
#define IN3 3 
#define IN4 4 
#define ENA 5    
#define ENB 6
# define Led1 9
# define Led2 10
# define Led3 11

typedef enum {
Idle_State,
Begin_Square,
Before_Maze,
MazeIn,
Turn90,
MazeOut,
ForceLeft,
Snake,
AfterWifi,
PitStop,
BeforeWhite,
BTHHH,
InWhite1,
InWhite2,
Finish,
Win
}States;

States State=0;
bool irMS,irRS,irLS,irERS,irELS;
bool onBlack=false,a=false,onWhite,b=false;

int PidStartSpeed=160,MaxSpeed=255,black=120;
float error=0,lastError=0,MinorError=0,integral,prevError;

int Kp=30,Kd=32,Ki=0;
int Ukp=24,Ukd=20,Uki=0;

float Fdist=100,Rdist=100,DistToKeep=7,FDistBeforeTurn=20;
int MazePidStartSpeed=80,MazeMaxSpeed=255;

unsigned long t1,tLed,t5,tPitOFF,tf,t2;

void setup() {
  // put your setup code here, to run once:

pinMode(USFE, INPUT);
pinMode(USRE, INPUT);
pinMode(USFT, OUTPUT);
pinMode(USRT, OUTPUT);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);
pinMode(ENA, OUTPUT);
pinMode(ENB, OUTPUT);
pinMode(Led1, OUTPUT);
pinMode(Led2, OUTPUT);
pinMode(Led3, OUTPUT);
digitalWrite(Led1,LOW);
digitalWrite(Led2,LOW);
digitalWrite(Led3,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

switch (State){
   case Idle_State:
      delay(500);
      State=State+1;
     // LedSignal();
   break;

   case Begin_Square:
      Ahead();
      while (irERS==1 || irELS==1)
        readBlack();
      State=State+1;
      //LedSignal();
   break;

   case Before_Maze:
      readBlack();
      
      if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0)){
        Rdist=readUltra(USRT , USRE);
        if (Rdist < 15)
         {State=State+1;
        // LedSignal();
         integral=0;prevError=0;
         }}
      GetError();
      PID();
   break;

   case MazeIn:
      Rdist=readUltra(USRT,USRE);
      Fdist=readUltra(USFT,USFE);
      MazePid();
      delay(30);
      Stop();
      
      if (Fdist<FDistBeforeTurn)
      {Stop();
      State=State+1;
      integral=0;prevError=0;error=0;}
      //LedSignal();
   break;

    case Turn90:
      delay(50);
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
      analogWrite(ENA,120);
      analogWrite(ENB,120);
      delay(310);
      Stop();
      delay(30);
      State=State+1;
      //LedSignal();   
   break;

   case MazeOut:
      Rdist=readUltra(USRT,USRE);
      MazePid();
      delay(20);
      Stop();
      readBlack();
      if ((irMS || irRS || irLS || irERS)&&(!irELS))
      {State=State+1;
      Kp=38;Kd=30;PidStartSpeed=70;
      integral=0;prevError=0;error=0;a=false;irELS=false;}
      //LedSignal();
   break;
   
   case ForceLeft:
         
      readBlack();
      GetError1();
      PID();
      if (irELS){
      Stop();
      delay(20);
      Ahead();
      delay(30);
      Stop();
      delay(20);
      LwStaticTurn();
      delay(300);
      Stop();
      delay(100);
      t5=millis();
      
      Kp=30;Kd=32;
      State=State+1;
      integral=0;prevError=0;  }
      //LedSignal(); 
   break;

  case Snake:
      readBlack();
      GetError1();

      if(millis()-t5>2800)
      {PidStartSpeed=50;
        PID();
      delay(12);
      Stop();
      delay(5);}
      else
      PID();

      if((irERS)&&(millis()-t5>3500)){
        State=State+1;
        Stop();
        delay(20);
        Ahead();
        delay(30);
        Stop();
        delay(20);

       while(!irELS){
        RwStaticTurn1();
        delay(16);
        Stop();
        delay(5);
        readBlack();}
        
        Stop();
        delay(100);
        RwStaticTurn1();
        delay(20);
        Ahead();
        delay(350);
        Stop();
        delay(30);
        
        integral=0;prevError=0;
        Kp=38;Kd=30;PidStartSpeed=120;irELS=false;irERS=false;
       // LedSignal();
        // maybe lower speed here
      }   

   break;

   case AfterWifi:
      readBlack();
      GetError();
      PID();
      if (irELS && irERS  && ((irRS) || (irLS)))
      {Ahead();
      delay(200); //delay to make sure the robot is in the middle of the pitstop
      Stop();
      State=State+1;}
     // LedSignal();     
   break;   

   case PitStop:
      digitalWrite(Led1,LOW);
      digitalWrite(Led2,LOW);
      digitalWrite(Led3,LOW);
      analogWrite(Led1,100);
      delay(2000);
      analogWrite(Led2,100);      
      delay(2000);
      analogWrite(Led3,100);
      delay(1000);
      digitalWrite(Led1,LOW);
      digitalWrite(Led2,LOW);
      digitalWrite(Led3,LOW);
      
     /*while((irELS)&&(irERS)){
      Ahead();}*/
      readBlack();
      LwStaticTurn2();
      delay(200);
       //while((irELS)&&(irERS))
      Ahead();
      delay(150);
      Stop();
      delay(50);
      //delay(200); //delay to get the robot out of the pitstop
      tPitOFF=millis();
      Kp = 30;
      Kd = 18;
      PidStartSpeed = 120;
      State=State+1;
     // LedSignal();
     break; 
   

    case BeforeWhite:
      readBlack();
      GetError();
     
      PID();
      if ((irELS) && (onBlack == false))
        onBlack = true;

      if (!(irELS)) {
        onBlack = false;
      }
      if ((irELS) && ((irERS)||(irRS))&&(millis()>700)) 
        ////Stop();
        {
          State = State + 1;
          Stop();
          delay(50);
          onBlack = false;
          //LedSignal();
        }
        break;

        case BTHHH:
        readWhite();

        analogWrite(ENA, 40);
        analogWrite(ENB, 40);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        
        if((irELS == 0) && (irLS == 0) && (irMS == 1) && (irRS == 0) && (irERS == 0))
        {Stop();
        delay(30);
        Ahead();
        delay(30);
        Stop();
        Kp = 30;
         Kd = 32;
         PidStartSpeed = 160;
        State=State+1;}
        
        
        break;

       case InWhite1:
      readWhite();
      GetError();  
      PID();

      if (irERS){
      Stop();
      delay(100);

      RwStaticTurn1();
      delay(470);
      Stop();
      delay(50);
      State=State+1;
      //LedSignal();
      onWhite=false;}
   break;  
     case InWhite2:
      readWhite();
      GetError();  
      PID();

      if (((irELS)&&(irERS))&&(onWhite==false))
      {tf=millis();onWhite=true;}
      if (!(((irELS)&&(irERS))))
      onWhite=false; 


      if((millis()-tf>80)&&((irELS)&&(irERS)))
      {State=State+1;
      PidStartSpeed=120;}
      //LedSignal();
   break;

   case Finish:
      readBlack();
      GetError4();
      
      if (((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))&&(millis()-t2>190))
      a=true;

      PID();

      if (((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))&&(onWhite==false))
      {onWhite=true;
      t2=millis();}

      if (!((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0)))
      {onWhite=false; t2=millis();a=false;}
      
      if ((irLS)&&(irRS)&&(irMS))
      {Ahead();
      delay(180);
      State=State+1;}
      //LedSignal();
   break; 



   case Win:
      Stop();
      analogWrite(Led1,100);
      analogWrite(Led2,100);      
      analogWrite(Led3,100);  
      delay(500);
      digitalWrite(Led1,LOW);
      digitalWrite(Led2,LOW);      
      digitalWrite(Led3,LOW);  
      delay(500); 
   break; 
}

if(millis()-tLed>300){
    digitalWrite(Led1,LOW);
    digitalWrite(Led2,LOW);      
    digitalWrite(Led3,LOW);
   }
}
   void readBlack(){
  irMS = analogRead(irSensorMiddle) > black;
  irRS = analogRead(irSensorRight) > black;
  irLS = analogRead(irSensorLeft) > black;
  irERS = analogRead(irSensorERight) > black;
  irELS = analogRead(irSensorELeft) > black;}

void readWhite(){
  irMS = analogRead(irSensorMiddle) < black;
  irRS = analogRead(irSensorRight) < black;
  irLS = analogRead(irSensorLeft) < black;
  irERS = analogRead(irSensorERight) < black;
  irELS = analogRead(irSensorELeft) < black;}


float readUltra(int Trigger,int Echo){
  digitalWrite(Trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);

  float duration = pulseIn(Echo, HIGH);
  return((duration*.0343)/2);}
  
void GetError(){

  if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==0) && (irERS==0))
  {error=0; a=false;}
  else if ((irELS==1) && (irLS==1) && (irMS==1) && (irRS==1) && (irERS==1))
  {error=0; a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==1))
  {error=3; a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==1) && (irERS==1))
  {error=3; a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==1) && (irERS==1))
  {error=2;a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==1) && (irERS==0))
  {error=2;a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==1) && (irERS==0))
  {error=1;a=false;}
  else if ((irELS==1) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=-3;a=false;}
  else if ((irELS==1) && (irLS==1) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=-3;a=false;} 
  else if ((irELS==1) && (irLS==1) && (irMS==1) && (irRS==0) && (irERS==0))
  {error=-2;a=false;}
  else if ((irELS==0) && (irLS==1) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=-2;a=false;}
  else if ((irELS==0) && (irLS==1) && (irMS==1) && (irRS==0) && (irERS==0))
  {error=-1;a=false;}
  else if ((irELS==0) && (irLS==1) && (irMS==1) && (irRS==1) && (irERS==0))
  {error=lastError;a=true;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=lastError;a=true;}
  else
  {error=MinorError;a=false;}

  if(error!=0)
  {lastError = error; // Update last error
  MinorError=(error/abs(error));}
  else
  MinorError=0;}

void GetError1(){

  if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==0) && (irERS==0))
  error=0;
  else if ((irELS==1) && (irLS==1) && (irMS==1) && (irRS==1) && (irERS==1))
  error=0;
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))
  error=0;
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==1))
  error=6;
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==1) && (irERS==1))
  error=5;
  else if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==1) && (irERS==1))
  error=4;
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==1) && (irERS==0))
  error=2;
  else if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==1) && (irERS==0))
  error=1;
  else if ((irELS==1) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))
  error=-6;
  else if ((irELS==1) && (irLS==1) && (irMS==0) && (irRS==0) && (irERS==0))
  error=-5;
  else if ((irELS==1) && (irLS==1) && (irMS==1) && (irRS==0) && (irERS==0))
  error=-4;
  else if ((irELS==0) && (irLS==1) && (irMS==0) && (irRS==0) && (irERS==0))
  error=-2;
  else if ((irELS==0) && (irLS==1) && (irMS==1) && (irRS==0) && (irERS==0))
  error=-0.5;
  else if ((irELS==0) && (irLS==1) && (irMS==1) && (irRS==1) && (irERS==0))
  error=lastError;
  else
  error=MinorError/2;

  if(error!=0)
  {lastError = error; // Update last error
  MinorError=(error/abs(error));}
  else
  MinorError=0;}  

void GetError4(){

  if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==0) && (irERS==0))
  {error=0; a=false;}
  else if ((irELS==1) && (irLS==1) && (irMS==1) && (irRS==1) && (irERS==1))
  {error=0; a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==1))
  {error=4; a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==1) && (irERS==1))
  {error=4; a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==1) && (irERS==1))
  {error=2;a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==1) && (irERS==0))
  {error=2;a=false;}
  else if ((irELS==0) && (irLS==0) && (irMS==1) && (irRS==1) && (irERS==0))
  {error=1;a=false;}
  else if ((irELS==1) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=-4;a=false;}
  else if ((irELS==1) && (irLS==1) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=-4;a=false;} 
  else if ((irELS==1) && (irLS==1) && (irMS==1) && (irRS==0) && (irERS==0))
  {error=-2;a=false;}
  else if ((irELS==0) && (irLS==1) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=-2;a=false;}
  else if ((irELS==0) && (irLS==1) && (irMS==1) && (irRS==0) && (irERS==0))
  {error=-1;a=false;}
  else if ((irELS==0) && (irLS==1) && (irMS==1) && (irRS==1) && (irERS==0))
  {error=lastError;}
  else if ((irELS==0) && (irLS==0) && (irMS==0) && (irRS==0) && (irERS==0))
  {error=MinorError;}
  else
  {error=MinorError;a=false;}

  if(error!=0)
  {lastError = error; // Update last error
  MinorError=(error/abs(error));}
  else
  MinorError=0;}



void PID(){
  integral += error; // Update integral

  // Calculate PID control output
  int pidOutput = Kp * error + Ki * integral + Kd * (error - prevError);
  
  prevError = error;

  // Calculate motor speeds
  int leftSpeed = PidStartSpeed + pidOutput;
  int rightSpeed = PidStartSpeed - pidOutput;

  // Ensure motor speeds are within limits
  leftSpeed = constrain(leftSpeed, 0, MaxSpeed);
  rightSpeed = constrain(rightSpeed,0, MaxSpeed);
  
  if(a){
    analogWrite(ENA, 60);
    analogWrite(ENB, 60);  
  if(lastError>0){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  }
  else{  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);   
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  }
  }
  else{
  analogWrite(ENA, leftSpeed);  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 

  analogWrite(ENB, rightSpeed);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  }
  }

void MazePid(){

  error= Rdist - DistToKeep;
  constrain(error,-2.5,2.5);
  
  integral += error; // Update integral

  // Calculate PID control output
  int pidOutput = Kp * error + Ki * integral + Kd * (error - prevError);
  
  prevError = error;

  // Calculate motor speeds
  int leftSpeed = MazePidStartSpeed + pidOutput;
  int rightSpeed = MazePidStartSpeed - pidOutput;

  // Ensure motor speeds are within limits
  leftSpeed = constrain(leftSpeed, 0, MazeMaxSpeed);
  rightSpeed = constrain(rightSpeed,0, MazeMaxSpeed);

  analogWrite(ENA, leftSpeed);  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 

  analogWrite(ENB, rightSpeed);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);}

void Ahead(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,MaxSpeed);
  analogWrite(ENB,MaxSpeed);}

void Stop(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);}

void RwStaticTurn(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,MaxSpeed);
  analogWrite(ENB,MaxSpeed);} 

  void RwStaticTurn1() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
}

void LwStaticTurn(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,MaxSpeed);
  analogWrite(ENB,MaxSpeed);} 

  void LwStaticTurn2(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,60);
  analogWrite(ENB,60);} 

void LedSignal(){
    analogWrite(Led1,100);
    analogWrite(Led2,100);      
    analogWrite(Led3,100);
    tLed=millis();}

