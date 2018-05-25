int pinOptico = 18;
int pinLin1 = 10;
int pinLin2 = 9;
int pinEnableDC= 3;
int pinMotDC1=4;
int pinMotDC2 =5;
int pinServo= 8;
int cuenta1para1Hoyo = 0;
bool inicio = true;
unsigned long t0;
unsigned long tact=0;
bool p = true;
double distancia = 22.5;
double v =0;
unsigned long numH = 0;
unsigned long valTimer =0;
#include <math.h> 
//double numH = 0;

//=====================
//SERVO
//=====================
#include <Servo.h>
Servo myservo;

///======================
//ROSS
//======================
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Float32 float_msg;
//std_msgs::String rec; 
ros::Publisher chatter("/plot_y160502", &float_msg);
void message_ros( const std_msgs::String& ros_msg );
ros::Subscriber <std_msgs::String> sub("msg_ejemplo", &message_ros);



void setup() {
  //PINES
  //pinMode(A0, INPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(pinOptico, INPUT);
  pinMode(pinLin1, INPUT);
  pinMode(pinLin2, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(pinOptico), lectura, RISING);
  //forw();
  
//ROSS
  nh.initNode();
  nh.advertise( chatter );
  nh.subscribe(sub);

  ///TIMER
  noInterrupts();
//  TCCR1B = 0;
//  TCCR1A = 0;
//  TCNT1 = 3036;//655536 -X = 3036
//  //X = 62500/Fdeseada
//  TCCR1B |= (1<<CS12); 
//  TIMSK1 |= (1 << TOIE1);
//set timer1 interrupt at 1Hz
  TCCR5A = 0;// set entire TCCR1A register to 0
  TCCR5B = 0;// same for TCCR1B
  TCNT5  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  //OCR2A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  //para decimas de segundo OCR1A = 1561
  //para centesimas de segundo OCR1A = 156;
  OCR5A = 1561;
  // turn on CTC mode
  TCCR5B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR5B |= (1 << CS12) | (1 << CS10); 
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE1A);

 interrupts();

 //SERVO
 // max servo 175
 //min servo 0
 pinMode(pinServo,OUTPUT);
 myservo.attach(pinServo);
 myservo.write(90);
  
 forw(150);
 //back();
}

void message_ros( const std_msgs::String& ros_msg ){
    //String rec = ros_msg.data;
    digitalWrite(2, HIGH);
    delay(3000);
    digitalWrite(2, LOW);
    str_msg.data = "Recibido";
    chatter.publish( &str_msg );
    nh.spinOnce();
  }


///Interrupcion de optointerruptor motor
void lectura(){

  numH++;
}

///Interrupcion de timer 1 decima de segundo
ISR(TIMER5_COMPA_vect){
  //Serial.println(numH);
  v = (numH/16.0);
  //Serial.println(v);
//  valTimer++;
  numH = 0;
// if(valTimer == 100)
// valTimer = 0;
}


//================
//Control motor DC
//================
void back(){
  digitalWrite(3,HIGH);
  analogWrite(4,25);
  analogWrite(5,0);
 
}


void forw(int in){
  digitalWrite(3,HIGH);

  if(in >=0){
    analogWrite(5,in);
    analogWrite(4,0);
    }
    else{
      in =-in;
      analogWrite(5,0);
      analogWrite(4,in);
      
      }
  
}
 

void stop1(){
  digitalWrite(3,LOW);
 
}


//================
//Servo
//================
int angulo =0;
  int i = 1;
  int band = 1;
  double k;

  void muevet(double ang, double seg){
    i = ang / abs(ang);
//    Serial.println(k);
//    Serial.println(k*1000);
    for(int j = 0; j < ang; j++){
      angulo+=i;
      if(angulo>=0 && angulo<=60){
        myservo.write(angulo);
        //Serial.println(angulo);
        delay(k*1000);
      }
    }
    band = 0;
  }

 void rangoServo(int minAn, int maxAn){
  
      int pos = minAn;
//    Serial.println(k);
//    Serial.println(k*1000);
  myservo.write(180);
 while(pos >= minAn && pos<= maxAn){
  pos = pos+i;
  myservo.write(angulo);
  delay(1000);
  if(pos == minAn || pos == maxAn)
    i = -i;
  
  }
  
  }
  
//estados:
  // 0 -> inicial
  // 1 -> vuelta izquierda
  // 2 -> vuelta derecha
int sigEdo ( int edo, int izq, int der){
  int sigEdo = edo;
    if( izq == 0 || der ==0){
    switch(edo) //donde opción es la variable a comparar
        {
            case 0:
              if(izq ==0)
              sigEdo = 1;
              else
              sigEdo = 2;
            break;
            case 1: 
              if(izq ==0)
                sigEdo = 1;
                else
                sigEdo = 0;//Bloque de instrucciones 2;
            break;
            case 2:
              if(izq ==0)
                sigEdo = 0;
                else
                sigEdo = 2;//Bloque de instrucciones 3;
            break;
            
//            default: //Bloque de instrucciones por defecto;
//            //default, es el bloque que se ejecuta en caso de que no se de ningún caso
        }
    }
    return sigEdo;   
  }

void accion(int edo){
  int alpha = 0;

  switch(edo){
    
    case 0:
      alpha = 90;
      break;
    case 1:
      alpha = 120;
      break;
    case 2:
      alpha = 60;
      break;
    }
  myservo.write(alpha);
//  if(edo == 0)
//    delay(100);
  }

double velD = 14;

double errorAcum = 0;
double ka = 1;
double kb = 0.5;
double ang = 35;
  double sec = 15;
  int izqL = 0;
  int derL = 0;
  int estado = 0;
 int in;
void loop() {

//Escribe velocidad en ross
  float_msg.data =  v;
  chatter.publish( &float_msg );
  nh.spinOnce();

////  float_msg.data =  valTimer;
////  if(valTimer %10 ==0)
////  chatter.publish( &float_msg );
//  //float_msg.data =  numH;
 
  //pinLin1 = der
  //pinLin2 = izq
  
 //Algoritmo seguidor de linea
 derL = digitalRead(pinLin1);
 izqL = digitalRead(pinLin2);
 estado = sigEdo(estado, izqL, derL);
 accion(estado);
 //delay(500);
  
// in = (int)(ka*(velD-v)+kb*errorAcum);
// forw(in);
// errorAcum = velD-v+errorAcum;


  
}
