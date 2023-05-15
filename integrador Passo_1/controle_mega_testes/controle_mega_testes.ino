#include <Wire.h>
#include <avr/pgmspace.h>
#define RPWM 5
#define LPWM 6
#define REN 8
#define LEN 9
#define b_home 12
#define leituras 1800


float passo=0.035;
int state=0;
int cont=0;
unsigned long tmp=0;
int tmp_degrau=0;
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
const PROGMEM float conv_ang=(2*M_PI)/4096;
const PROGMEM float conv_pos=(M_PI*0.024)/1600;
const PROGMEM float conv_sai=255/24;
float target= 0;
float target_1= 0;
float e=0;
float senoide=0;
const PROGMEM float T=0.008224;
const PROGMEM float K[5]={ -0.42672,-58.6293,-26.3512,-12.5266,1};
const PROGMEM float Aa[2][2]={
  { 5.0652e-01,  -1.5926e-08},
  { 7.9538e-01 ,  5.0630e-01},
};
const PROGMEM float Bp[2]={ 0.037011,-0.059652} ;
const PROGMEM float Ke[2][2]={
  { 1.5407e-07,  -3.2041e-03},
  {-2.8808e-03 ,  6.0113e+01},
};
const PROGMEM float Ab[2][2]={
  {-1.5407e-07,  -2.5595e-03},
  { 2.8808e-03 , -5.9968e+01},
};
float Xv[2]={0,0};
float Xv_1[2]={0,0};
float uo=0;
float uo_1=0;  
float integrador=0;
float integrador_1=0;
float dif=0;
float dif_1=0;

  



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
  tmp_degrau=0;
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
  
 

//  setMotor(1,30,RPWM,LPWM,REN,LEN);
//  while(digitalRead(b_home)==0);
//  setMotor(-1,30,RPWM,LPWM,REN,LEN);
//  delay(500);
//  setMotor(1,20,RPWM,LPWM,REN,LEN);
//  while(digitalRead(b_home)==0);
//  setMotor(-1,30,RPWM,LPWM,REN,LEN);
//  delay(500);  
//  setMotor(0,0,RPWM,LPWM,REN,LEN);
//  delay(5000);


  TIMSK2 = (TIMSK2 & B11111110) | 0x01;
  TCCR2B = (TCCR2B & B11111000) | 0x06;


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

//    Serial.print(u2.LePos,DEC);
//    Serial.print(",");
//    Serial.println(u1.LePos,DEC);

  
  if(pwr==0&&dir==0){Serial.println("ok");}
  if(pwr==255&& state==0){
    TIMSK2 = (TIMSK2 & B11111110) | 0x00;
    Serial.println("Erro");
    state=1;
   }
  

  
  //if(target!=target_1){Serial.println(target);}
  //target_1=target;
}

ISR(TIMER2_OVF_vect){
  
  tmp++;
  tmp_degrau++;
  cont=500;
  int dez_sec=9727;
  if(tmp_degrau<cont){target=0;}
//  if(tmp>cont){target=0.06*sin(0.1*M_PI*T*(tmp-500));}
  if(tmp_degrau>cont&&tmp_degrau<dez_sec){target=passo;}
  if(tmp_degrau>dez_sec&&tmp_degrau<2*dez_sec){target=-passo;}
  //if(tmp_degrau>2*dez_sec){target=0;}
  if(tmp_degrau>2*dez_sec){tmp_degrau=500;}


//  senoide=sin(2*M_PI*T*(tmp));



  
  
  pos=u2.LePos*conv_pos;
  ang=u1.LePos*conv_ang;

Xv[0]=Aa[0][0]*Xv_1[0]+Aa[0][1]*Xv_1[1]+ Bp[0]*uo_1 + Ke[0][0]*pos + Ke[0][1]*ang + Ab[0][0]*pos_1 + Ab[0][1]*ang_1;
Xv[1]=Aa[1][0]*Xv_1[0]+Aa[1][1]*Xv_1[1]+ Bp[1]*uo_1 + Ke[1][0]*pos + Ke[1][1]*ang + Ab[1][0]*pos_1 + Ab[1][1]*ang_1;


dif=target-pos;
integrador=integrador_1+(0.5*T)*(dif*T+dif_1);


 uo =  - (K[4]*integrador +pos*K[0]+ang*K[1]+Xv[0]*K[2]+Xv[1]*K[3]);
  
 e=uo;
 
  pos_1=pos;
  ang_1=ang;
  Xv_1[0]=Xv[0];
  Xv_1[1]=Xv[1];
  integrador_1=integrador;
  dif_1=dif;
  uo_1=uo;



  pwr = abs(e*conv_sai); 
  if( pwr > 255 ){pwr = 255;}
  if( pwr < 0 ){pwr = 0;} 
  if(e<0){dir = 1;}  // motor direction
  if(e>0){dir = -1;}  // motor direction

if(u2.LePos>2800 || u2.LePos<-2800){
  dir=0;
  pwr=0;
  TIMSK2 = (TIMSK2 & B11111110) | 0x00;
  }

setMotor(dir,pwr,RPWM,LPWM,REN,LEN);


if(tmp>cont+1 && u2.LePos<2500 && u2.LePos>-2500 && pwr<255 && pwr>=0){
    Serial.print(tmp,DEC);
    Serial.print(",");
    Serial.print(u2.LePos,DEC);
    Serial.print(",");
    Serial.print(u1.LePos,DEC);
    Serial.print(",");
    Serial.print(Xv[0],DEC);
    Serial.print(",");
    Serial.print(Xv[1],DEC);
    Serial.print(",");
    Serial.print(uo,DEC);
    Serial.print(",");
    Serial.println(target,DEC);
}

// Serial.print(u1.LePos, DEC );
//  Serial.print(pos,DEC);
//  Serial.print(";");
//  Serial.print(u2.LePos, DEC );
//  Serial.print(senoide, DEC );
//  Serial.println();
  
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
