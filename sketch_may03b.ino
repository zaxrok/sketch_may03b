
/*  
 *  코드스타: Power scratch externsion block
 *  (주)이산솔루션, www.isans.co.kr
 */

#include <Wire.h> // I2C Arduino Library
#include <IRremote.h>

// for codestar
#define FORWARD   0
#define REVERSE   1
#define MOTOR_L   0
#define MOTOR_R   1

//////////////////////////////////// geometric sensor ////////////////////////////////////
#define HMC5883L_ADDR   0x1E // 0011110b, I2C 7bit address of HMC5883
bool haveHMC5883L = false;

//////////////////////////////////// IRRemote ////////////////////////////////////
const int pinRemocon = 4;
IRrecv remocon(pinRemocon);
decode_results results;

//////////////////////////////////// enum ////////////////////////////////////
enum deviceNumber{
  SONAR = 1,
  BUTTON = 2,
  VARIABLER = 3,
  MIC = 4, 
  TEMP = 5, 
  MOTION = 6, 
  GYRO = 7, 
  GEOM = 8, 
  TOUCH = 9, 
  LIGHT = 10, 
  IR = 11, 
  
  COLOR0 = 21, 
  COLOR3 = 22, 
  BUZZER = 23, 
  BUZZERSTOP = 24, 
  SERVO = 25, 
  IRR = 26, 
  VIBRATION = 27,
  WHEEL = 28,
  DRIVE = 29,
 
  
  VERSION = 41, 
  TIMER = 42,
};
enum protocol{
  GET = 1,
  RUN = 2,
  RESET = 4, 
  START = 5
};

//////////////////////////////////// for test ////////////////////////////////////
const int pinRed = 7;  // 빨간눈(Red LED) 연결 핀 번호
const int pinBlue = 8; // 파란눈(Blue LED) 연결 핀 번호
const int pinBuzzer = A3;
const int tones[13] = {196,220,247,262,294,330,349,392,440,494,523,587,659};
//////////////////////////////////// defined pin ////////////////////////////////////
// MOTOR_LEFT
int pinDirL = 2;
int pinSpeedL = 5;

// MOTOR_RIGHT
int pinSpeedR = 6;
int pinDirR = 3;

// sonar
const int pinEcho = A0;  // 초음파 센서 Echo 단자 연결 핀 번호
const int pinTrig = 13;  // 초음파 센서 Trig 단자 연결 핀 번호

//////////////////////////////////// union ////////////////////////////////////
union{
  byte byteVal[2];
  short shortVal;
}valShort;

union{
  byte byteVal[4];
  float floatVal;
}valFloat;

//////////////////////////////////// global variable ////////////////////////////////////
#if defined(__AVR_ATmega328P__) or defined(__AVR_ATmega168__)
  int analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};
#endif
double  currentTime = 0.0; // on request TIMER
double  lastTime = 0.0;    // on request TIMER
boolean isAvailable = false;  // serial
boolean isStart = false;  // fine package
String  mVersion = "1.0.0";
byte    dataLen = 0;      // package data length;
char    serialRead;       // real data
unsigned char prevc = 0;  // real data
byte    index = 0;        // buffer index
char    buffer[52];       // real buffer
int preDistance = 0;      // 초음파 -1 때문에
//////////////////////////////////// send data to serial ////////////////////////////////////
void sendByte(char c){
  writeSerial(1); // byte 
  writeSerial(c);
}
void sendShort(int value){
  writeSerial(2);   // short
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}
void sendFloat(float value){
  writeSerial(3);   // float
  valFloat.floatVal = value;
  writeSerial(valFloat.byteVal[0]);
  writeSerial(valFloat.byteVal[1]);
  writeSerial(valFloat.byteVal[2]);
  writeSerial(valFloat.byteVal[3]);
}
void writeHead(){
  writeSerial(0xff);
  writeSerial(0x55);
}
void writeEnd(){
  Serial.println();
}
void writeSerial(unsigned char c){
  Serial.write(c);
}
void callOK(){
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

//////////////////////////////////// read data by serial ////////////////////////////////////
void readSerial(){
  isAvailable = false;
  if(Serial.available() > 0){
    isAvailable = true;
    serialRead = Serial.read();
  }
}

//////////////////////////////////// buffer function ////////////////////////////////////
short readShort(int idx){
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal;
}
unsigned char readBuffer(int idx){
  return buffer[idx];
}

void writeBuffer(int idx, unsigned char c){
  buffer[idx] = c;
}
//////////////////////////////////// Geometric function ////////////////////////////////////
bool detectHMC5883L ()
{
  // read identification registers
  Wire.beginTransmission(HMC5883L_ADDR); // HMC5883과 통신 시작
  Wire.write(10); // select Identification register A
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 3);
  
  if(Wire.available() == 3) {
    char a = Wire.read();
    char b = Wire.read();
    char c = Wire.read();
    
    if(a == 'H' && b == '4' && c == '3')
      return true;
  }

  return false;
}
//////////////////////////////////// sonar function ////////////////////////////////////
int  GetDistance(){
  
  digitalWrite( pinTrig, LOW ); 
  delayMicroseconds( 2 );   
  digitalWrite( pinTrig, HIGH );
  delayMicroseconds( 10 );  
  digitalWrite( pinTrig, LOW ); 
  int  duration = pulseIn( pinEcho, HIGH );
  int  distance = (duration / 2) / 29.1;   
  if(distance < 0 ) distance = preDistance;
  preDistance = distance;
  return  distance;
}
/*
ff 55 len x  GET  sensor  port  slot  data a
0  1  2   3   4      5      6     7     8
*/
void runModule(int device){
  int port = readBuffer(6);
  int value = 0;
  int pin = 0;
  int dir = 0;
  switch(device){
    case VIBRATION:
    case COLOR0:
      pinMode(port, OUTPUT);
      value = readBuffer(7);
      delay(10);
      digitalWrite(port, value);
    break;
    case COLOR3:
      pinMode(port, OUTPUT);
      value = readBuffer(7);
      delay(10);
      analogWrite(port, value);
    break;
    case BUZZER:
      pin = analogs[port];
      pinMode(pin, OUTPUT);
      value = readShort(7);
      delay(10);
      tone(pin, value);
     break;
     case BUZZERSTOP:
      pin = analogs[port];
      delay(10);
      noTone(pin);
     break;
     
     case SERVO: break; 
     
     case DRIVE:
      dir = readBuffer(7);// dir  forward 0, backward 1, left 2, right 3
      value = readBuffer(8); // 0 ~ 255
      if(dir == 2){ // left
        move(MOTOR_L, REVERSE, value);
        move(MOTOR_R, FORWARD, value); 
      }
      else if(dir == 3){  // right
        move(MOTOR_L, FORWARD, value);
        move(MOTOR_R, REVERSE, value); 
      }
      else if(dir == 1){ // backward
        move(MOTOR_L, REVERSE, value);
        move(MOTOR_R, REVERSE, value); 
      }
      else if(dir == 0){
        move(MOTOR_L, FORWARD, value);
        move(MOTOR_R, FORWARD, value); 
      }
     break;
     
     case WHEEL: 
      dir = readBuffer(7);  // left 0, right 1
      value = readShort(8);    // 255 ~ -255
      if(value < 0)
        move(dir, REVERSE, value*-1);
      else
        move(dir, FORWARD, value);
     break;
  }
}
void readSensor(int device){
  int port = readBuffer(6);
  int value = 0;
  int oC = 0; // Temperature
  switch(device){
    case SONAR:
      sendShort(GetDistance());  // 거리 측정
    break;
    
    case BUTTON:
    case TOUCH:
      pinMode(port, INPUT);
      delay(10);
      sendByte(digitalRead(port)?0:1);
    break;
    
    case TEMP:
      pinMode(analogs[port], INPUT);
      delay(10);
      value = analogRead(analogs[port]);  // 온도 읽기
      oC = (5.0 * value * 100.0) / 1024.0; // 섭씨온도로 변환(LM35)
      sendByte(oC);
    break;
    
    case MOTION:
    break;
    
    case GYRO:
    break;
    
    case GEOM:
     {
        bool detect = detectHMC5883L();
        if(!haveHMC5883L){
          if(detect){
            haveHMC5883L = true;
            // Put the HMC5883 IC into the correct operating mode
            Wire.beginTransmission(HMC5883L_ADDR); // HMC5883과 통신 시작
            Wire.write(0x02); // select mode register
            Wire.write(0x00); // continuous measurement mode
            Wire.endTransmission();
          }
          else{ delay(20); break; } // try
        }

        int axis[3];//x, y, z; //triple axis data
        // Tell the HMC5883 where to begin reading data
        Wire.beginTransmission(HMC5883L_ADDR);
        Wire.write(0x03); // select register 3, X MSB register
        Wire.endTransmission();
      
       // Read data from each axis, 2 registers per axis
        Wire.requestFrom(HMC5883L_ADDR, 6);
        
        if(Wire.available() >= 6) {
          axis[0] = Wire.read() << 8; // X msb
          axis[0] |= Wire.read(); // X lsb
          axis[2] = Wire.read() << 8; // Z msb
          axis[2] |= Wire.read(); // Z lsb
          axis[1] = Wire.read() << 8; // Y msb
          axis[1] |= Wire.read(); // Y lsb
        }
        int pos = readBuffer(7); 
        sendShort(axis[pos]);
     }
    break; 

    case VARIABLER:
    case LIGHT:
    case MIC:
    case IR:
      pinMode(analogs[port], INPUT);
      delay(10);
      sendShort(analogRead(port)); // 0~1023
    break;  

    case IRR:
      if (remocon.decode(&results)){  // 받은 신호가 있나? (받고나면 수신 차단됨)
        sendByte(results.value);
        remocon.resume(); // 다음 리모콘 신호를 수신하는 상태로
      }
    break;

    case VERSION: break;  
    case TIMER: break; 

  }
}
/*
ff 55 len idx action device port  slot  data a
0  1  2   3   4      5      6     7     8
*/
void parseData(){
  int idx = readBuffer(3);
  int action = readBuffer(4);
  int device = readBuffer(5);

  switch(action){
    case GET:
      writeHead();
      writeSerial(device);
      readSensor(device);
      writeEnd();
    break;
    case RUN:
      runModule(device);
      callOK();
    break;
    case RESET:
    break;
    case START:
    break;
  }
}

void move(int motor, int direction, int speed){
  boolean inPin1, inPin2;

  if(direction == FORWARD)
    inPin1 = HIGH;
  else // REVERSE
    inPin1 = LOW;

  if(motor == MOTOR_L){
    digitalWrite(pinDirL, inPin1);
    analogWrite(pinSpeedL, speed);
  } else { // MOTOR_R
    digitalWrite(pinDirR, inPin1);
    analogWrite(pinSpeedR, speed);
  }
}

void setup() {
  // mobile
  pinMode(pinSpeedL, OUTPUT);
  pinMode(pinDirL, OUTPUT);

  pinMode(pinSpeedR, OUTPUT);
  pinMode(pinDirR, OUTPUT);

  pinMode(pinRed, OUTPUT);
  pinMode(pinBlue, OUTPUT);
  
  // sonar
  pinMode(pinTrig, OUTPUT); // 출력용 핀으로 설정
  pinMode(pinEcho, INPUT);  // 입력용 핀으로 설정
        
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.flush();

  pinMode(pinBuzzer, OUTPUT); // 부저(스피커) 핀을 출력용 핀으로 설정

  int title[] = { 330, 294, 392 };
  tone(pinBuzzer, 247);
  digitalWrite(pinRed, HIGH);
  digitalWrite(pinBlue, HIGH);
  delay(500);
  digitalWrite(pinRed, LOW);
  digitalWrite(pinBlue, LOW);
  for(int i = 0; i < 3; i++){
      tone(pinBuzzer, title[i]);
      delay(300);
      digitalWrite(pinRed, HIGH);
      digitalWrite(pinBlue, HIGH);
  }
  digitalWrite(pinRed, LOW);
  digitalWrite(pinBlue, LOW);
  noTone(pinBuzzer);

 
  remocon.enableIRIn(); // IR 리모콘 신호 수신 시작

  // geometric sensor
  TWBR = 78;  // lower I2C clock : 25 kHz 
  TWSR |= _BV (TWPS0);  // change prescaler  
}

void loop() { 
  // put your main code here, to run repeatedly:
  currentTime = millis()/1000.0-lastTime;
  readSerial();
  if(isAvailable){
    unsigned char c = serialRead & 0xff;
    if(c == 0x55 && isStart == false){
      if(prevc == 0xff){
        index = 1;
        isStart = true;
      }
    }else{  // isn't 0x55
      prevc = c;
      if(isStart){  // finded header code
        if(index == 2){ // is index length?
          dataLen = c;
        }else if(index > 2){
          dataLen--;
        }
        writeBuffer(index, c);    // real value
      }
    }

    index++;
    if(index > 51){ // checking max buffer
      index = 0;
      isStart = false;
    }

    if(isStart && dataLen == 0 && index > 3){
      isStart = false;    // reset
      parseData();
      index = 0;
    }
  }
}
