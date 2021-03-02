/*
*      THE ZG 接收机 主程序
*           BY:微风萧萧
*                
* 
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_PWMServoDriver.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//  遥杆1 遥杆2  遥杆3  遥杆4 电位器A 电位器B
int cha1, cha2, cha3, cha4,  cha5, cha6;
// 开关A 开关B  开关R 开关L
int swA, swB,  swR, swL;
// 定义数据变量
int mode =0,Rexo,Reup,Redo,cont;
int addr;

Servo ch1; 
Servo ch2; 
Servo ch3; 
Servo ch4; 
Servo ch5;

struct Signal {
  byte vlx;
  byte vly;
  byte vrx;
  byte vry;
  byte potentialA;      //电位器a
  byte potentialB;      //电位器b
  byte switchA;         //开关a   ------>F->1001   B->1110
  byte switchB;         //开关b   ------>O->1011   X->1100
  byte touchA;          //触摸开关a  ------>NL->1010   LI->1000
  byte touchB;          //触摸开关b  ------>NL->1111   LI->0000
};
Signal data;

const byte address[6] = "zg111";
RF24 radio(7, 8);      // CE, CSN 

void setup() {
  ch1.attach(3);
  ch2.attach(5);
  ch3.attach(6);
  ch4.attach(9);
  ch5.attach(10);

  pwm.begin();         //PCA9685初始化
  pwm.setPWMFreq(50);  //50HZ
  
  Serial.begin(9600);  //打开串口
  radio.begin();       //nrf24l01p 初始化
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pinMode(A0,INPUT);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(A3,OUTPUT);
  mode =  EEPROM.read(addr);
  cont = HIGH;
  pinMode(LED_BUILTIN,OUTPUT);//LED推挽输出
  digitalWrite(LED_BUILTIN,HIGH);
  digitalWrite(A1,HIGH);
  digitalWrite(A2,HIGH);
  digitalWrite(A3,HIGH);
}
unsigned long lastRecvTime = 0;
void recvData(){
  while( radio.available() ) {
    radio.read(&data, sizeof(Signal));//接收数据
    lastRecvTime = millis();   //当前时间ms
  }
}
void loop() {
   recvData();
   unsigned long now = millis();
   if ( now - lastRecvTime > 500 ) {digitalWrite(LED_BUILTIN,HIGH);}else digitalWrite(LED_BUILTIN,LOW);
    Rexo = analogRead(A0);
    Reup = analogRead(A6);
    Redo = analogRead(A7);

    swA = data.switchA;
    swB = data.switchB;
    swR = data.touchA;
    swL = data.touchB;
    if(Rexo <= 20&&cont == HIGH){  //模式转换
              delay(4);
              mode++;
              if(mode >=4){
                  mode =0;
                }
              cont =!cont;
      }else if(Rexo >=1000 &&cont ==LOW){
               delay(4);
               cont =!cont;
        }
   /*  if(Reup <= 20&&cont == HIGH){       //暂时没想好
              delay(4);
              
              cont =!cont;
      }else if(Reup >=1000 &&cont ==LOW){
               delay(4);
               cont =!cont;
        }
     if(Redo <= 20&&cont == HIGH){        //暂时没想好
              delay(4);
              
              cont =!cont;
      }else if(Redo >=1000 &&cont ==LOW){
               delay(4);
               cont =!cont;
        }*/
    if(mode==0){ 
       digitalWrite(A1,1); digitalWrite(A2,1); digitalWrite(A3,1);
    }else if(mode==1){      //  蓝色  模式一
                  cha1 = map(data.vlx, 0, 255, 1000, 2000);   // 
                  cha2 = map(data.vly, 0, 255, 185,370);      // 
                  cha3 = map(data.vrx, 0, 255, 1000, 2000);   // 
                  cha4 = map(data.vry, 0, 255, 1000, 2000);   // 
                  cha5 = map(data.potentialA, 0, 255, 185, 370);      //50Hz 无刷电调上升沿为1ms-2ms 对应值为187-373
                  cha6 = map(data.potentialB, 0, 255, 185, 370);      //舵机上升沿102为0度 187为45度 280为90度 373为135度 510为180度 
              digitalWrite(A1,0);
              digitalWrite(A3,1);
               pwm.setPWM(12,0,cha5);
               pwm.setPWM(13,0,cha6);
               if(cha2<=310&&cha2>=245){
                    pwm.setPWM(4,0,227);
                   }else pwm.setPWM(4,0,cha2);
            } 
    else if(mode==2){  //绿色   模式二
                  cha1 = map(data.vlx, 0, 255, 1000, 2000);   // 
                  cha2 = map(data.vly, 0, 255, 950, 1900);    // 
                  cha3 = map(data.vrx, 0, 255, 1000, 2000);   // 
                  cha4 = map(data.vry, 0, 255, 1000, 2000);   // 
                  cha5 = map(data.potentialA, 0, 255, 185, 370);    
                  cha6 = map(data.potentialB, 0, 255, 185, 370);     
              digitalWrite(A1,1);
              digitalWrite(A2,0);
               ch1.writeMicroseconds(cha1);  // 3号引脚 lx
               ch2.writeMicroseconds(cha2);  // 5号引脚 ly
               ch3.writeMicroseconds(cha3);  // 6号引脚 rx
               ch4.writeMicroseconds(cha4);  // 9号引脚 ry
               //ch5.writeMicroseconds(cha2);  //1000 --2000 对应1ms-2ms
           }
    else if(mode==3){  //红色   模式三
              digitalWrite(A2,1);
              digitalWrite(A3,0);
              
            }
    EEPROM.write(addr, mode);  
    delay(50);
}
