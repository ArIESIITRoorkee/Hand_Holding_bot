#include <SoftwareSerial.h>

//#include <SoftwareSerial.h>
//SoftwareSerial Serial1(8,9);//rx,tx
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free  sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
float angle=0;
float base=0;
float Error=0;

float kpy=1;
float kdy=1;
String Path="";
float correction=0;
char ch="";
int com=0;
int FR1= 28;
int FL1=24;
int FR2=30;
int FL2=26;
int BR1=36;
int BL1=32;
int BR2=38;
int BL2=34;
int FLS=8;
int BLS=11;
int FRS=9;
int BRS=6;
long int pwmfl=100;
long int pwmfr=100;
long int pwmbl=100;
long int pwmbr=100;
long int fl=0;
long int fr=0;
long int bl=0;
long int br=0;
float pError;
float dError;
long int Thresh1;
long int Thresh2;



long int d1;
long int d2;
long int d3;

const int trigPin1=51;
const int echoPin1=53;
long duration_1;
int distance_1;
const int trigPin2=43;
const int echoPin2=41;
long duration_2;
int distance_2;
const int trigPin3=31;
const int echoPin3=33; 
long duration_3;
int distance_3;

//SoftwareSerial Serial1(9,10);

//int grid[30];
bool reached=false;
int queue[35];
int m = 0;
int a,b,c;
bool visited[30];
int pred[30];
int e,f;
int x,y;
int grid[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//int grid[30] = {1,1,0,1,1,1,1,0,1,1,1,0,1,0,0,1,0,1,1,1,1,0,0,0,1,0,1,0,0,0 };
//int grid[30];
String pathh="";

////////////////////////////////////////
int queue1[40];
int a1,b1,c1;
bool visited1[17];
int pred1[17];
int locate[17][4];
int e1,f1;
bool reached1 = false;
String pathMain = "";
int direct = 0;
int triggerPin = 15;//for dest
int currentPos = 0;
int trigCheck;// for checkpoint
int g;
int trigg=2; // for grid
int dir;
////////
void initvis1(){
  for(int i = 0;i<17;i++) visited1[i] = false;
}
void initpred1(){
  for(int i =0;i<17;i++) pred1[i] = -1;
}
void initqueue1(){
  a1 = 0;
  b1 = 0;
  for(int i = 0;i<35;i++) queue1[i] = -1;
}
void push1(int d){
  queue1[b] = d;
  b1 = b1 + 1;
}
int pop1(){
  a1 = a1 + 1;
  return queue1[a1-1];
}
bool view1(){
  if(queue1[a1] == -1) return false;
  else return true;
}
void shortPath1(int src1,int dest1){
    push1(dest1);
    visited1[dest1] = true;
    
    while((view1()) && (a1<=b1) && (!reached1)){
      c1 = pop1();
      for (int i = 0;i<4;i++){
         if(locate[c][i] != -1){
        if(!(visited1[locate[c1][i]])) {
          pred1[locate[c1][i]] = c1;
          push1(locate[c1][i]);
          visited1[locate[c1][i]] = true;
          if(locate[c1][i] == src1){
            reached1 = true;
            return;
          }
        }
      }
    }
    }
}
void followpath1(int src1, int dest1){
  e1 = src1;
  Serial.println("path");
  while(pred1[e1] != dest1){
    for(int i = 0;i<4;i++){
      if(locate[e1][i] == pred1) pathMain += i;
    }

     e1 = pred1[e1];
  }
  g = pathMain.length();
} 
void short_algo1(int src1,int dest1)
{pathMain ="";
reached1 = false;
   initvis1();
  initpred1();
  initqueue1();
  shortPath1(src1,dest1);
followpath1(src1,dest1);
}


////////////////////////////////////////
int distance1(){
 while(true) 
{digitalWrite(trigPin1, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin1, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin1, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration_1 = pulseIn(echoPin1, HIGH);
// Calculating the distance
distance_1= duration_1*0.034/2;
// Prints the distance on the Serial Monitor
Serial.print("Distance_1: ");
Serial.println(distance_1);
return distance_1;
}
  
  }



  int distance2(){
 while(true)
{digitalWrite(trigPin2, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin2, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin2, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration_2 = pulseIn(echoPin2, HIGH);
// Calculating the distance
distance_2= duration_2*0.034/2;
// Prints the distance on the Serial Monitor
Serial.print("Distance_2: ");
Serial.println(distance_2);
return distance_2;
}
  
  }

    int distance3(){
 while(true)
{digitalWrite(trigPin3, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin3, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin3, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration_3 = pulseIn(echoPin3, HIGH);
// Calculating the distance
distance_3= duration_3*0.034/2;
// Prints the distance on the Serial Monitor
Serial.print("Distance_3: ");
Serial.println(distance_3);
return distance_3;
}
  
  }
 



void String_read_kr(){
//  while(!Serial1.available()){
//    
//  }
//  for(int j = 5;j<30;j++){
//      char ch = Serial1.read();
//      grid[j] = int(ch)-48;
//    }
//Serial1.flush();
//
//  digitalWrite(trigg, HIGH);
//  Serial1.println("high");
//  int i=5;
//  while(i<30){
//  while(!Serial.available()){
//    
//  }
//    while(Serial.available()){
//      char ch = Serial.read();
//      if(ch!=' '){
//        grid[i] = (int)ch-48;
//        i++;
//        if(i==30) break;
//      }
//    }
//  }
//  digitalWrite(trigg, LOW);
//  Serial.flush();
////  Serial1.println(grid);
//  Serial1.flush();



digitalWrite(trigg, HIGH);
  Serial1.println("high");
  while(Path.length()<25){
  while(!Serial.available()){
    
  }
    while(Serial.available()){
      char ch = Serial.read();
      if(ch!=' '){
        Path = Path+ch;
        if(Path.length()==25) break;
      }
    }
  }
  
  Serial.print("Path = ");
  Serial1.println(Path);
  for(int i=0; i<25;i++){
    grid[i+5] = (int)Path.charAt(i)-48;
  }
  
  Serial1.print("grid = ");
  for(int i=0; i<30; i++){
    Serial1.print(grid[i]);
  }
  
  Serial1.println();
  digitalWrite(trigg, LOW);
  Serial.flush();
  Path = "";
  Serial1.flush();
  grid[4]=1;
  grid[3]=1;
  grid[0]=1;
  grid[1]=1;
//
//  if(distance_1>=Thresh1){
//    if(distance_1>=Thresh2){
//      grid[4] = 1;
//    }
//    grid[3] = 1;
//  }
//
//  if(distance_3>=Thresh1){
//    if(distance_3>=Thresh2){
//      grid[0] = 1;
//    }
//    grid[1] = 1;
//  }
}
void String_read(){
  while(!Serial1.available()){
    int dest1 = Serial1.read();
  }
  Serial1.flush();
}



void initvis(){
  for(int i = 0;i<30;i++) visited[i] = false;
}
void initpred(){
  for(int i =0;i<30;i++) pred[i] = -1;
}
void initqueue(){
  a = 0;
  b = 0;
  for(int i = 0;i<35;i++) queue[i] = -1;
}
void push(int d){
  queue[b] = d;
  b = b + 1;
}
int pop(){
  a = a + 1;
  return queue[a-1];
}
bool view(){
  if(queue[a] == -1) return false;
  else return true;
}
void getcood(int c){
  y = (int)(c/5);
  x = c-5*y;
}
void shortPath(int src,int dest){
    push(dest);
    visited[dest] = true;
    
    while((view()) && (a<=b) && (!reached)){
      c = pop();
      getcood(c);
      if(y-1>-1){
       if((grid[c-5] == 1) & !visited[c-5]){
        pred[c-5] = c;
        push(c-5);
        visited[c-5] = true;
        if(c-5 == src) {
          reached = true;
          break;        
        }
       }
      }
      
      if(x-1>-1){
       if((grid[c-1] == 1) & !visited[c-1]){
        pred[c-1] = c;
        push(c-1);
        visited[c-1] = true;  
       if(c-1 == src) {
          reached = true;
          break;        
        }
       }
      }
      if(x+1<5){
       if((grid[c+1] == 1) & !visited[c+1]){
        pred[c+1] = c;
        push(c+1);
        visited[c+1] = true;  
       if(c+1 == src) {
          reached = true;
          break;        
        }
       }
      }
      if(y+1<6){
       if((grid[c+5] == 1) & !visited[c+5]){
        pred[c+5] = c;
        push(c+5); 
        visited[c+5] = true; 
       if(c+5 == src) {
          reached = true;
          break;        
        }
       }
      }

  }
 for(int i = 0;i<30;i++){
 Serial.println(pred[i]);
 }
}
void followpath(int src, int dest){
  e = src;
  Serial.println("path");
  while((pred[e] != dest) & (pred[e] != -1)){
    f = pred[e] - e;
    if (f==1) pathh += 'r';
    else if(f==-1) pathh += 'l';
    else if(f==5) pathh += 'f';
    else if(f==-5)pathh += 'b';
     e = pred[e];
  }
pathh += 'e';
}  
void short_algo(int src,int dest){
  pathh = "";
reached = false;
  initvis();
  initpred();
  initqueue();
  shortPath(src,dest);
followpath(src,dest);
}

void findDest(){
  if(com == 0){
    for(int j = 0;j<3;j++){
    for(int i = 0;i<2;i++){
      short_algo(2,27+i-5*j);
      if(reached) {
        com += i;
        return;
      }
    }
    short_algo(2,26-5*j);
    if(reached) {
        com += -1;
        return;
      }
  }
}
  if(com == 1){
    for(int j = 0;j<3;j++){
    for(int i = -1;i<2;i++){
      short_algo(2,27+i-5*j);
      if(reached){
        com += i;
        return;
      }
    }
  }
}

  if(com == -1){
    for(int j = 0;j<3;j++){
    for(int i = 1;i>-2;i--){
      short_algo(2,27+i-5*j);
      if(reached){
        com += i;
        return;
      }
    }
  }
}
}

void initLocate(){
  for(int i = 0;i < 17;i++){
    for(int j = 0;j<4;j++){
      locate[i][j] = -1;
    }
  }
}

void setup(){
 Serial.begin(9600); 
Serial1.begin(9600);
digitalWrite(trigg, LOW);
  pinMode(FR1,OUTPUT);
   pinMode(FR2,OUTPUT);
    pinMode(FL1,OUTPUT);
     pinMode(FL2,OUTPUT);
      pinMode(BL1,OUTPUT);
       pinMode(BL2,OUTPUT);
        pinMode(BR1,OUTPUT);
         pinMode(BR2,OUTPUT);
          pinMode(FLS,OUTPUT);
           pinMode(BLS,OUTPUT);
           pinMode(FRS,OUTPUT);
           pinMode(BRS,OUTPUT);
           pinMode(trigPin1,OUTPUT);
           
pinMode(echoPin1,INPUT);
pinMode(trigPin2,OUTPUT);
pinMode(echoPin2,INPUT);
pinMode(trigPin3,OUTPUT);
pinMode(echoPin3,INPUT);
  pinMode(triggerPin, INPUT_PULLUP);
  
  //*****************************
  initLocate();
  locate[0][0] = 1;
locate[1][2] = 0;
locate[1][3] = 2;
locate[1][0] = 3;
locate[2][1] = 1;
locate[3][1] = 4;
locate[3][0] = 16;
locate[4][3] = 3;
locate[4][1] = 5;
locate[5][3] = 4;
locate[5][1] = 6;
locate[5][2] = 7;
locate[6][3] = 5;
locate[6][0] = 11;
locate[6][2] =9;
locate[7][0] = 5;
locate[7][1] = 9;
locate[7][2] = 8;
locate[8][0] = 7;
locate[8][1] = 10;
locate[9][0] = 6;
locate[9][2] = 10;
locate[9][3] = 7;
locate[10][3] = 8;
locate[10][0] = 9;
locate[11][2] = 6;
locate[11][3] = 12;
locate[12][1] = 11;
locate[12][3] = 13;
locate[13][1] = 12;
locate[13][2] = 14;
locate[14][0] = 13;
locate[14][3] = 15;
locate[15][1] = 14;
locate[15][2] = 16;
locate[16][0] = 15;
locate[16][2] = 3;
  //*****************************
//  attachInterrupt(0, loop1, RISING);
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  while (!Serial); 
   Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
//    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        //dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    
  int t = millis();
  while(millis()-t<20000){
    Err();
    Serial.println(millis()-t);
    Serial.print("angle: ");
    Serial.println(angle);
  }
  base = angle;


}

void Err(){
  mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaterni on(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
        #endif
        angle = ypr[0]*180/M_PI;
    }   
}




void forward(){

Err();
  Error = angle-base;
  dError = Error - pError;
  correction = kpy*pError + kdy*dError;
  pError=Error; 
  Serial.print("correction=");
  Serial.println(correction);
  fl=pwmfl+correction;
  fr=pwmfr-correction;
  bl=pwmbl-correction;
  br=pwmbr+correction;

    if (fl>255)
  fl=255;
    if (bl>255)
  bl=255;
    if (fr>255)
  fr=255;
    if(br>255)
  br=255;
//  analogWrite(mf,mod(sf));
//  analogWrite(ml,mod(sl));
//  analogWrite(mr,mod(sr));
//  Serial.print(sl);
//  Serial.print(" ");
//  Serial.print(sr);
//  Serial.print(" ");
//  Serial.println(sf);

  
 digitalWrite(BL1,HIGH);
  digitalWrite(BL2,LOW);
  digitalWrite(BR1,HIGH);
 digitalWrite(BR2,LOW);
  analogWrite(BLS,bl);
   analogWrite(BRS,br);
    digitalWrite(FL1,HIGH);
  digitalWrite(FL2,LOW);
  digitalWrite(FR1,HIGH);
 digitalWrite(FR2,LOW);
  analogWrite(FLS,fl);
   analogWrite(FRS,fr);
 }
 void left(){


Err();
  Error = angle-base;
  dError = Error - pError;
  correction = kpy*pError + kdy*dError;
  pError=Error; 
  Serial.print("correction=");
  Serial.println(correction);
  fl=pwmfl-correction;
  fr=pwmfr-correction;
  bl=pwmbl+ correction;
  br=pwmbr+correction;

    if (fl>255)
  fl=255;
    if (bl>255)
  bl=255;
    if (fr>255)
  fr=255;
    if(br>255)
  br=255;
 digitalWrite(BL2,HIGH);
  digitalWrite(BL1,LOW);
  digitalWrite(BR2,LOW);
 digitalWrite(BR1,HIGH);
  analogWrite(BLS,bl);
   analogWrite(BRS,br);
    digitalWrite(FL2,LOW);
  digitalWrite(FL1,HIGH);
  digitalWrite(FR2,HIGH);
 digitalWrite(FR1,LOW);
  analogWrite(FLS,fl);
   analogWrite(FRS,fr);

 }
 void right(){


  Err();
  Error = angle-base;
  dError = Error - pError;
  correction = kpy*pError + kdy*dError;
  pError=Error; 
  Serial.print("correction=");
  Serial.println(correction);
  fl=pwmfl+correction;
  fr=pwmfr+correction;
  bl=pwmbl-correction;
  br=pwmbr-correction+30;

    if (fl>255)
  fl=255;
    if (bl>255)
  bl=255;
    if (fr>255)
  fr=255;
    if(br>255)
  br=255;
  digitalWrite(FL1,LOW);
  digitalWrite(FL2,HIGH);
    digitalWrite(FR1,HIGH);
  digitalWrite(FR2,LOW);
    digitalWrite(BL1,HIGH);
  digitalWrite(BL2,LOW);
    digitalWrite(BR1,LOW);
  digitalWrite(BR2,HIGH);
  analogWrite(FLS,fl);
  analogWrite(BLS,bl);
  analogWrite(FRS,fr);
  analogWrite(BRS,br);
  
 }
 void backward(){

  Err();
  Error = angle-base;
  dError = Error - pError;
  correction = kpy*pError + kdy*dError;
  pError=Error; 
  Serial.print("correction=");
  Serial.println(correction);
  fl=pwmfl+correction;
  fr=pwmfr-correction;
  bl=pwmbl+correction;
  br=pwmbr-correction;

    if (fl>255)
  fl=255;
    if (bl>255)
  bl=255;
    if (fr>255)
  fr=255;
    if(br>255)
  br=255;

    digitalWrite(FL1,LOW);
  digitalWrite(FL2,HIGH);
    digitalWrite(FR1,LOW);
  digitalWrite(FR2,HIGH);
    digitalWrite(BL1,LOW);
  digitalWrite(BL2,HIGH);
    digitalWrite(BR1,LOW);
  digitalWrite(BR2,HIGH);
  analogWrite(FLS,fl);
  analogWrite(BLS,bl);
  analogWrite(FRS,fr);
  analogWrite(BRS,br);


 }
void Stop(){
   analogWrite(FLS,60);
  analogWrite(BLS,60);
  analogWrite(FRS,60);
  analogWrite(BRS,60); 
  }


void loop(){

   
  
pinMode(trigg,HIGH);
Serial.println("step1");


String_read_kr();
Serial.println("step2");

pinMode(trigg,LOW);

findDest();

Serial.println("step3");

Serial.print("path=");
Serial.println(pathh);
int c=pathh.length();
Serial.print("length=");
Serial.println(c);
for(int k=0;k<=c;k++){
if(pathh.charAt(k)==' '){
    k++;
  }
  ch = pathh.charAt(k);
  Serial.print("character=");
  Serial.println(ch);
  if(ch == 'f')
  {Serial.println("forward");
    int t = millis();
    while(millis()-t<=1000){
      forward();
        //Serial.println(value);
      }
  }
  if (ch == 'l')
  {Serial.println("left");
    int t = millis();
    while(millis()-t<=1000){
      left();
        //Serial.println(value);
      }}
  
  if (ch == 'r')
  {Serial.println("right");
    int t = millis();
    while(millis()-t<=1000){
      right();
        //Serial.println(value);
      }}
  if (ch == 'b')
  {Serial.println("back");
  
    int t = millis();
    while(millis()-t<=1000){
      backward();
        //Serial.println(value);
      }
     
  }
  
  if (ch == 'e')
  {Serial.println("stop");

  int t = millis();
    while(millis()-t<=1000){
      Stop();
        //Serial.println(value);
      }
    
    pathh="";
   
    }
  }
}
void loop1(){
  if(digitalRead(triggerPin)){                                 //If trigger pin from raspi is on for destination. Integrated with chatbot
    String_read;                                               //To read destination    
short_algo1(currentPos,dest1);//get  the path
  while(currentPos =! dest1){
    for(int i = 0;i<g;i++){
      dir = int(pathMain[i]) - 48 - direct ;
      direct = int(pathMain[i]) - 48;
      if(dir < 0) dir += 4;
      if(dir==0)
      {turn 0}
      if(dir==1)
      turn cw
      if dir==3
      turn acw;
      if dir==2
      reverse 180
            
      /// g is string length and direction is according to axis of bot;
      while(!digitalRead(trigCheck)){
//        
//      
      digitalWrite(trigg,HIGH);
      String_read_kr();
      digitalWrite(trigg,LOW);
      findDest();
Serial.println("step3");

Serial.print("path=");
Serial.println(pathh);
int c=pathh.length();
Serial.print("length=");
Serial.println(c);
for(int k=0;k<=c;k++){
  
if(pathh.charAt(k)==' '){
    k++;
  }
  ch = pathh.charAt(k);
  Serial.print("character=");
  Serial.println(ch);
  if(ch == 'f')
  {Serial.println("forward");
    int t = millis();
    while(millis()-t<=1000){
      
      forward();
      if(digitalRead(trigCheck)){
        break;
      }
        //Serial.println(value);
      }
  }
  if (ch == 'l')
  {Serial.println("left");
    int t = millis();
    while(millis()-t<=1000){
      left();
        //Serial.println(value);
        if(digitalRead(trigCheck)){
        break;
      }
    }
  }
  
  if (ch == 'r')
  {Serial.println("right");
    int t = millis();
    while(millis()-t<=1000){
      right();
        //Serial.println(value);
        if(digitalRead(trigCheck)){
        break;
      }
      }}
  if (ch == 'b')
  {Serial.println("back");
  
    int t = millis();
    while(millis()-t<=1000){
      backward();
      if(digitalRead(trigCheck)){
        break;
      }
        //Serial.println(value);
      }
     
  }
  
  if (ch == 'e')
  {Serial.println("stop");

  int t = millis();
    while(millis()-t<=1000){
      Stop();
      
        //Serial.println(value);
      }
    
    pathh="";
   
    }
  }
      
      //move according to pathh
      //Interrupt
//      if(trigCheck) {
      }
        currentPos = pred1[currentPos];
//        continue;
      
    }
  }
  } 
}
