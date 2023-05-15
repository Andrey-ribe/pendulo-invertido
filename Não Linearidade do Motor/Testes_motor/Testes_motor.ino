#include <Wire.h>
#include <avr/pgmspace.h>
#define RPWM 5
#define LPWM 6
#define REN 8
#define LEN 9
#define b_home 12
#define leituras 1800
#define passo -0.04


long ePos = 0; // encoder position
int state=0;
int cont=0;
int tmp=0;
int ciclos=0;
int pwr;
int dir=0;
int vetor[2][leituras];
float pos=0;
float pos_1=0;
float ang=0;
float ang_1=0;
float vel=0;
float vel_ang=0;
const PROGMEM float conv_ang=(2*M_PI)/4000;
const PROGMEM float conv_pos=(M_PI*0.024)/1600;
const PROGMEM float conv_sai=255/24;
float target= 0;
float target_1= 0;
float e=0;
float senoide=0;
const PROGMEM float T=0.002048;
const PROGMEM float K[4]={-4.9496 , -63.8044  ,-28.7235 , -13.6388};
const PROGMEM float Aa[2][2]={
  {8.4418E-01 , -3.6819E-07},
  {3.1840E-01  , 8.4350E-01},
};
const PROGMEM float Bp[2]={0.011686 , -0.023880};
const PROGMEM float Ke[2][2]={
   {1.9093E-07 , -7.5594E-04},
   {-7.4492E-04 ,  7.6440E+01},
};
const PROGMEM float Ab[2][2]={
  {-1.9093E-07 , -1.0641E-03},
  { 7.4492E-04 , -7.6396E+01},
};
float Xv[2]={0,0};
float Xv_1[2]={0,0};
float uo=0;
float uo_1=0;


union u_tag1 {  // allow long to be read as 4 seperate bytes
   byte b[4];   // 4 bytes to be received over I2C
   long LePos;  // encoder position as 4 byte long
} u1;

union u_tag2 {  // allow long to be read as 4 seperate bytes
   byte b[4];   // 4 bytes to be received over I2C
   long LePos;  // encoder position as 4 byte long
} u2;

void setup() {
  tmp=0;
  ciclos=0;
  pinMode(b_home,INPUT);
  pinMode(13,OUTPUT);
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(LEN,OUTPUT);
  pinMode(REN,OUTPUT);
  
  Wire.setClock(400000);
  Wire.begin();         // join i2c bus (address optional for master)
  Serial.begin(115200); // start serial for output
  
  TIMSK2 = (TIMSK2 & B11111110) | 0x01;
  TCCR2B = (TCCR2B & B11111000) | 0x04;

  Serial.println("Angu;Posica");
}

void loop()
{

    Wire.requestFrom(8, 4);    // request 6 bytes from slave device #8
  
  for (int i=0; i<4; i++) //requestFrom() is a looping code; it terminates when all requested has come :: thanks GolamMostafa!!
  {
      u1.b[i] = Wire.read();  //data bytes come from FIFO Buffer that has been filled up by requestFrom()
  }

  Wire.requestFrom(7, 4);    // request 6 bytes from slave device #8
  
  for (int i=0; i<4; i++) //requestFrom() is a looping code; it terminates when all requested has come :: thanks GolamMostafa!!
  {
      u2.b[i] = Wire.read();  //data bytes come from FIFO Buffer that has been filled up by requestFrom()
  }
//  Serial.print(u2.LePos,DEC);
//  Serial.print(",");
//  Serial.println(pwr,DEC);
  
//  if(pwr==0&&dir==0){Serial.println("ok");}
//  if(pwr==255&& state==0){Serial.println("Erro");}
  
  if(ciclos>(leituras-1) && state==0){
    
      TIMSK2 = (TIMSK2 & B11111110) | 0x00;
      setMotor(0,0,RPWM,LPWM,REN,LEN);
    Serial.println("VETOR(contagem,posição,angulo)");
    
      for (int i=0; i<leituras; i++){
        Serial.print(vetor[0][i]);
        Serial.print(",");
        Serial.println(vetor[1][i]);
      }
      
    state=7;
    }
  
  if(target!=target_1){Serial.println(target);}
  target_1=target;
}

ISR(TIMER2_OVF_vect){





  if(ciclos<=leituras){
    vetor[0][ciclos]=ciclos;
    vetor[1][ciclos]=u2.LePos;
    ciclos++;
    }
  
  
  pos=u2.LePos*conv_pos;
  ang=u1.LePos*conv_ang;

  
  e=24;
 




  pwr = abs(e*conv_sai); 
  if( pwr > 255 ){pwr = 255;}
  if( pwr < 0 ){pwr = 0;} 
  if(e<0){dir = 1;}  // motor direction
  if(e>0){dir = -1;}  // motor direction

if(u2.LePos>5000 || u2.LePos<0){dir=0;pwr=0;}

setMotor(dir,pwr,RPWM,LPWM,REN,LEN);

  
}

void setMotor(int dir, int pwmVal, int rpwm, int lpwm, int ren, int len){

  if(dir == 1){
    digitalWrite(ren,HIGH);
    analogWrite(rpwm,pwmVal);
    digitalWrite(len,HIGH);
    analogWrite(lpwm,0);
    
  }
  else if(dir == -1){
    digitalWrite(ren,HIGH);
    analogWrite(rpwm,0);
    digitalWrite(len,HIGH);
    analogWrite(lpwm,pwmVal);
  }
  else{
    analogWrite(rpwm,0);
    digitalWrite(ren,LOW);
    analogWrite(lpwm,0);
    digitalWrite(len,LOW);
  }  


}
