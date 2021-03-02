/*
  THE ZG NANO遥控器
          by 微风萧萧
      引脚分配：
          A0 A1 A2 A3 遥杆4通道 
          A5 A6 OLED液晶屏SCL SDA
          A7 A8 电位器1 电位器2
          D2 D3 D4 D5 4触摸按键 2自锁 2点动
          D6 PWM LED 输出
          D7 D8 机械开关 SW1 SW2
          nrf24l01p:
            CE  -> D9  CSN -> D10  MOSI -> D11   MISO -> D12
            SCK -> D13 IRQ -> \  VCC  -> 3.3V GND  -> GND
  
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE);
static const unsigned char PROGMEM bitmap[] ={      //59*31
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xF8,0x7F,0x00,0x00,0x7F,0x00,0x00,0x80,0xFF,0x7F,0x00,0xC0,0xFF,0x00,
0x00,0xF8,0xFF,0x7F,0x00,0xE0,0xFF,0x00,0x00,0xF8,0x7F,0xFC,0x00,0xF8,0xF7,0x00,
0x00,0xF8,0x01,0x7E,0x00,0xFC,0xE1,0x00,0x00,0x00,0x00,0x3F,0x00,0x7E,0x00,0x00,
0x00,0x00,0x80,0x1F,0x00,0x3F,0x00,0x00,0x00,0x00,0xC0,0x0F,0x80,0x0F,0x00,0x00,
0x00,0x00,0xE0,0x07,0xC0,0x07,0x00,0x00,0x00,0x00,0xF8,0x03,0xE0,0x03,0x00,0x00,
0x00,0x00,0xFC,0x00,0xE0,0x01,0xFC,0x00,0x00,0x00,0x7E,0x00,0xF0,0x81,0xFF,0x00,
0x00,0x00,0x3F,0x00,0xF0,0xF0,0xFF,0x01,0x00,0x00,0x1F,0x00,0x78,0xF8,0xFF,0x01,
0x00,0xC0,0x0F,0x00,0x78,0xF8,0xF3,0x00,0x00,0xE0,0x07,0x00,0x3C,0x78,0xF8,0x00,
0x00,0xF8,0x03,0x00,0x3C,0x00,0xF8,0x00,0x00,0xFC,0x00,0x00,0x3C,0x00,0x7C,0x00,
0x00,0x7E,0x00,0x3C,0x3C,0x00,0x3E,0x00,0x00,0x3F,0x00,0x3E,0x3C,0x00,0x3F,0x00,
0x80,0x1F,0x80,0x3F,0x7C,0x80,0x1F,0x00,0xC0,0x0F,0xF0,0x1F,0xFC,0xE0,0x0F,0x00,
0xE0,0xFF,0xFF,0x07,0xF8,0xF9,0x07,0x00,0xF0,0xFF,0xFF,0x03,0xF8,0xFF,0x01,0x00,
0xF0,0xFF,0x7F,0x00,0xF0,0xFF,0x00,0x00,0xF8,0xFF,0x07,0x00,0xC0,0x1F,0x00,0x00,
0xF0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const unsigned char PROGMEM bitmapa[] ={0x0C,0x1D,0x27,0x36,0x1C,0x1C,0x36,0x27,0x1D,0x0C};   // 蓝牙图标6*9


RF24 radio(9, 10); // CE, CSN
const byte address[6] = "zg111"; //定义通讯地址

struct Signal {
  byte vlx;
  byte vly;
  byte vrx;
  byte vry;
  byte potentialA;      //电位器a
  byte potentialB;      //电位器b
  byte switchA;         //开关a   ------>F->1001   B->1110
  byte switchB;          //开关b   ------>O->1011   X->1100
  byte touchA;          //触摸开关a  ------>NL->1010   LI->1000
  byte touchB;          //触摸开关b  ------>NL->1111   LI->0000
};
Signal data;

int SWC1,SWC2,SWC3,SWC4;
int SW5,SW6;            //系统操作按键
int cont1,cont2,cont3,cont4;        //按键消抖中间变量
int mode;   //系统模式更换变量 mode=0 操作模式 mode=1设置模式
int ym;     //oled页面变量 ym=1,2,3
int Bluetooh;  //蓝牙开启,默认开启
char sreialData; // 储存蓝牙指令

void page1() {
  u8g2.setFont(u8g2_font_timB18_tf);
  u8g2.setFontPosTop();
  u8g2.setCursor(2,2);
  u8g2.print("THE ZG");
  u8g2.drawHLine(1,21,126); //直线 （x，y，长度）
  u8g2.setFont(u8g2_font_helvR08_tf);
  u8g2.setFontPosTop();
  u8g2.drawFrame(1,23,78,22); //矩形框 （起点x，起点y，长，宽）
  
  u8g2.setCursor(2,24);u8g2.print("LX:");u8g2.setCursor(18,24);u8g2.print(data.vlx);
  u8g2.setCursor(40,24);u8g2.print("RX:");u8g2.setCursor(57,24);u8g2.print(data.vrx);
  u8g2.setCursor(2,34);u8g2.print("LY:");u8g2.setCursor(18,34);u8g2.print(data.vly);
  u8g2.setCursor(40,34);u8g2.print("RY:");u8g2.setCursor(57,34);u8g2.print(data.vry);

  u8g2.setCursor(2,45);u8g2.print("ReU:");u8g2.setCursor(26,45);u8g2.print(data.potentialA);
  u8g2.setCursor(48,45);u8g2.print("RxD:");u8g2.setCursor(73,45);u8g2.print(data.potentialB);
  u8g2.setCursor(85,24);u8g2.print("SW1:");     //-------------------------
  if (SWC1 == 1) {u8g2.setCursor(112,24);u8g2.print("F");
  } else {u8g2.setCursor(112,24);u8g2.print("B");}
  u8g2.setCursor(85,34);u8g2.print("SW2:");     //----------------------
  if (SWC2 == 1) {u8g2.setCursor(112,34);u8g2.print("X");
  } else {u8g2.setCursor(112,34);u8g2.print("O");}
  u8g2.setCursor(94,44);u8g2.print("S3:");      //----------------------
  if (SWC3 == 1) {u8g2.setCursor(112,44);u8g2.print("T");
  } else {u8g2.setCursor(112,44);u8g2.print("K");}
  u8g2.setCursor(94,54);u8g2.print("S4:");      //----------------------
  if (SWC4 == 1) {u8g2.setCursor(112,54);u8g2.print("R");
  } else {u8g2.setCursor(112,54);u8g2.print("C");}
  u8g2.setFont(u8g2_font_ncenR08_tf); 
  u8g2.setFontPosTop();
  u8g2.setCursor(2,54);u8g2.print("[Back]");      //----------------------
}
void page2() {
      u8g2.drawXBMP(34, 2, 59, 31, bitmap);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.setFontPosTop();
      if(Bluetooh ==1){ u8g2.drawXBMP(120, 2, 6, 10, bitmapa);}else{ u8g2.setCursor(120,2);u8g2.print("  ");}
      if(mode ==1){  u8g2.setCursor(2,42);u8g2.print("Setup"); }else if(mode ==0){ u8g2.setCursor(2,42);u8g2.print("Operation");}
      u8g2.setFont(u8g2_font_ncenR08_tf); 
      u8g2.setFontPosTop();
      u8g2.setCursor(2,54);u8g2.print("[Select]"); 
      u8g2.setCursor(80,54);u8g2.print("[DataSita]"); 
}
void page3() {
      u8g2.setFont(u8g2_font_timB18_tf);
      u8g2.setFontPosTop();
      u8g2.setCursor(2,2);
      u8g2.print("THE ZG");
      u8g2.drawHLine(1,21,126); //直线 （x，y，长度）
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.setFontPosTop();
      u8g2.setCursor(44,24);u8g2.print("Sorry,"); 
      u8g2.setCursor(12,34);u8g2.print("no time,come out, OK"); 
      u8g2.setCursor(5,44);u8g2.print(">"); 
      if (Bluetooh ==1){
            u8g2.setCursor(12,44);u8g2.print("Turn off the Bluetooth?"); 
        }else if(Bluetooh ==0 ){
          u8g2.setCursor(12,44);u8g2.print("Turn on the Bluetooth?");
          }
      u8g2.setFont(u8g2_font_ncenR08_tf); 
      u8g2.setFontPosTop();
      u8g2.setCursor(2,54);u8g2.print("[Back]"); 
      u8g2.setCursor(102,54);u8g2.print("[OK]"); 
}

void setup(){
  radio.begin();
  radio.openWritingPipe(address);// 写入地址
  radio.setPALevel(RF24_PA_MIN); //小功率
  radio.stopListening();  //nrf24l01设为发射模式
  
  Serial.begin(9600);   //打开串口接收蓝牙数据
  
  u8g2.begin();
  u8g2.enableUTF8Print();
  pinMode(6, OUTPUT);     //led指示灯
  pinMode(7, INPUT);      //通道开关1
  pinMode(8, INPUT);       //通道开关2
  pinMode(2, INPUT);
  pinMode(3, INPUT);       //通道开关3
  pinMode(4, INPUT);
  pinMode(5, INPUT);        //通道开关4

  mode=0;
  ym=2;
  cont1 = HIGH;
  cont2 = HIGH;
  cont3 = HIGH;
  cont4 = HIGH;
  Bluetooh = 1;    //开启蓝牙  =0时关闭串口接收
  
  if (analogRead(A6)>20||analogRead(A7)>20){    //检测电位器初始值,不为零时led提示
    for(int i=0;i<3;i++){
      analogWrite(6,0);//led闪烁3次
      delay(500);
      analogWrite(6,250);
      delay(500);
    }
  } else{
    digitalWrite(6,0);//led闪烁1次
    delay(500);
    digitalWrite(6,250);
  }
}

int chValue(int val, int lower, int middle, int upper)
{
  val = constrain(val, lower, upper);     //将val限制在lower~upper范围内
  if ( val < middle )
    val = map(val, lower, middle, 0, 127);
  else
    val = map(val, middle, upper, 128, 255);
  return val;
}

void loop(){
  SW5 = digitalRead(2);
  SW6 = digitalRead(4);

    if( Bluetooh == 1){
      if (Serial.available() > 0) {
            delay(5);
            sreialData = Serial.read();
            
            if (sreialData == 'S') {    //设置模式
                  mode=1; digitalWrite(6,0);
          } else if (sreialData == 'P') { //操作模式
                    mode = 0;digitalWrite(6,255);
          } else if (sreialData == 'M') {  //页面1
                    ym=1;
          } else if (sreialData == 'U') {   //页面2
                    ym=2;
          }else if (sreialData == 'N') {   //页面3
                    ym=3;
          }
          delay(4);
           Serial.println(sreialData);
          }
       }
  
  if(SW5 == 1&&SW6 == 1&&cont3==HIGH){
              delay(5);
      if(mode==0){delay(10);mode = 1 ;digitalWrite(6,0);} 
      else if(mode==1){delay(10);mode =0;digitalWrite(6,255);}
    
    }else if(SW5 == 0&&SW6 == 0&&cont3==LOW){
           delay(5);
           cont3 = !cont3;
      }
  
  if(mode ==0){
        SWC1 = digitalRead(7);
        SWC2 = digitalRead(8);
        SWC3 = digitalRead(3);
        SWC4 = digitalRead(5);
        data.vlx =255- chValue(analogRead(A0), 150, 478, 900);
        data.vly = chValue(analogRead(A1), 140, 505, 910);
        data.vrx = chValue(analogRead(A2), 160, 492, 900);
        data.vry =255- chValue(analogRead(A3), 140, 513, 910);
        data.potentialA = map(analogRead(A6),0, 1023, 0, 255); 
        data.potentialB = map(analogRead(A7),0, 1023, 0, 255); 
        if(SWC1 ==1){ data.switchA = 1001 ;}else{data.switchA = 1110 ;}
        if(SWC2 ==1){ data.switchB = 1100 ;}else{data.switchB = 1011 ;}
        if(SWC3 ==1){ data.touchA = 1000 ;}else{data.touchA = 1010 ;}
        if(SWC4 ==1){ data.touchB = 1101 ;}else{data.touchB = 1111 ;}
        radio.write(&data, sizeof(Signal));
        
    }else if(mode ==1){
        if(ym==1){
             if (SW6 == HIGH && cont2 == HIGH) {
                    delay(5);
                    ym =2;
                    cont2 = !cont2;
                  } else if (SW6 == LOW && cont2 == LOW) {
                    delay(5);
                    cont2 = !cont2;
                  }
          }else if(ym==2){
                  if (SW6 == HIGH && cont2 == HIGH) {
                    delay(5);
                    ym =3;
                    cont2 = !cont2;
                  } else if (SW6 == LOW && cont2 == LOW) {
                    delay(5);
                    cont2 = !cont2;
                  }
                  if (SW5 == HIGH && cont1 == HIGH) {
                      delay(5);
                      ym = 1;
                      cont1 = !cont1;
                    } else if (SW5 == LOW && cont1 == LOW) {
                      delay(5);
                      cont1 = !cont1;
                    }
            }else if(ym ==3){
                  if (SW6 == HIGH && cont2 == HIGH) {
                    delay(5);
                    ym =2;
                    cont2 = !cont2;
                  } else if (SW6 == LOW && cont2 == LOW) {
                    delay(5);
                    cont2 = !cont2;
                  }
                  if (SW5 == HIGH && cont1 == HIGH) {
                      delay(5);
                     if(Bluetooh == 1){
                        delay(5);
                        Bluetooh =0;
                      }else if(Bluetooh ==0){
                        delay(5);
                        Bluetooh =1;
                        }
                      cont1 = !cont1;
                    } else if (SW5 == LOW && cont1 == LOW) {
                      delay(5);
                      cont1 = !cont1;
                    }
              }
         }

  //---
  u8g2.firstPage();
        do{
          if(ym==3){page3();}
          else if(ym==2){page2();}
          else if(ym==1){page1();}
        }
  while (u8g2.nextPage());
delay(50);
}
