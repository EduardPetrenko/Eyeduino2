/***********************************************************************************
February 2014, E.Petrenko
video load into Arduino (Eyeduino project)

Find mass and mass center of black pixels (we consider image to be an image of
"cockroach". If mass is less then MASS_IGNORE then we do not move. If mass is between MASS_IGNORE 
and MASS_AVOID then we attack spot. If mass is greater then MASS_AVOID, then Robot tries to
avoid the spot. If spot mass center is near target area of image, weapon (servo) rotates to "kick cockroach".

In this version servo action was added if mass center of the spot is in the target 
region of hunting "weapon".
***********************************************************************************/


//Command code
#define COMMAND_SWITCH_MODE 0
#define COMMAND_RUN	10
#define COMMAND_MOTORS_STOP	11
#define COMMAND_ANALOG_WRITE	20
#define COMMAND_DIGITAL_WRITE	30
#define COMMAND_ANALOG_READ	40
#define COMMAND_ANALOG_READ_RE	41
#define COMMAND_DIGITAL_READ	50
#define COMMAND_DIGITAL_READ_RE	51
#define COMMAND_READ_IR	60
#define COMMAND_READ_IR_RE	61
#define COMMAND_ACTION_DONE 70
#define COMMAND_READ_TRIM 80
#define COMMAND_READ_TRIM_RE 81
#define COMMAND_PAUSE_MODE 90
#define COMMAND_LINE_FOLLOW_CONFIG 100

//component codename
#define CN_LEFT_MOTOR	0
#define CN_RIGHT_MOTOR	1
#define CN_IR 2

//motor board modes
#define MODE_SIMPLE 0
#define MODE_LINE_FOLLOW 1
#define MODE_ADJUST_MOTOR 2
#define MODE_IR_CONTROL 3

#define details(name) (byte*)&name,sizeof(name)

//Not neccessary, but just in case. 
#if ARDUINO > 22
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "HardwareSerial.h"
//#include <NewSoftSerial.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>

class EasyTransfer3 {
public:
void begin(HardwareSerial *theSerial);
//void begin(uint8_t *, uint8_t, NewSoftSerial *theSerial);
void sendData();
boolean receiveData();

void writeByte(uint8_t dat);
void writeInt(int dat);
uint8_t readByte();
int readInt();


private:
HardwareSerial *_serial;

void resetData();

uint8_t data[20];	//data storage, for both read and send
uint8_t position;
uint8_t size; //size of data in bytes. Both for read and send
uint8_t rx_array_inx;  //index for RX parsing buffer
uint8_t rx_len;		//RX packet length according to the packet
uint8_t calc_CS;	   //calculated Chacksum
};


//Captures address and size of struct
void EasyTransfer3::begin(HardwareSerial *theSerial){
	_serial = theSerial;
	resetData();
}

void EasyTransfer3::writeByte(uint8_t dat){
	if(position<20)
		data[position++]=dat;
		size++;
}
void EasyTransfer3::writeInt(int dat){
	if(position<19){
		data[position++]=dat>>8;
		data[position++]=dat;
		size+=2;
	}
}
uint8_t EasyTransfer3::readByte(){
	if(position>=size)return 0;
	return data[position++];
}
int EasyTransfer3::readInt(){
	if(position+1>=size)return 0;
	int dat_1=data[position++]<<8;
	int dat_2=data[position++];
	int dat= dat_1 | dat_2;
	return dat;
}

void EasyTransfer3::resetData(){
	for(int i=0;i<20;i++){
		data[i]=0;
	}
	size=0;
	position=0;
}

//Sends out struct in binary, with header, length info and checksum
void EasyTransfer3::sendData(){
  uint8_t CS = size;
  _serial->write(0x06);
  _serial->write(0x85);
  _serial->write(size);
  for(int i = 0; i<size; i++){
    CS^=*(data+i);
    _serial->write(*(data+i));
  }
  _serial->write(CS);
  
  resetData();
}

boolean EasyTransfer3::receiveData(){
  
  //start off by looking for the header bytes. If they were already found in a previous call, skip it.
  if(rx_len == 0){
  //this size check may be redundant due to the size check below, but for now I'll leave it the way it is.
    if(_serial->available() >= 3){
	//this will block until a 0x06 is found or buffer size becomes less then 3.
      while(_serial->read() != 0x06) {
		//This will trash any preamble junk in the serial buffer
		//but we need to make sure there is enough in the buffer to process while we trash the rest
		//if the buffer becomes too empty, we will escape and try again on the next call
		if(_serial->available() < 3)
			return false;
		}
      if (_serial->read() == 0x85){
        rx_len = _serial->read();
		resetData();

		//make sure the binary structs on both Arduinos are the same size.
      }
    }
  }
  
  //we get here if we already found the header bytes, the struct size matched what we know, and now we are byte aligned.
  if(rx_len != 0){
	
    while(_serial->available() && rx_array_inx <= rx_len){
      data[rx_array_inx++] = _serial->read();
    }
    
    if(rx_len == (rx_array_inx-1)){
      //seem to have got whole message
      //last uint8_t is CS
      calc_CS = rx_len;
      for (int i = 0; i<rx_len; i++){
        calc_CS^=data[i];
      } 
	  
      if(calc_CS == data[rx_array_inx-1]){//CS good
		//resetData();
		for(int i=0;i<20;i++){
		}
		size=rx_len;
		rx_len = 0;
		rx_array_inx = 0;
		return true;
		}
		
	  else{
		resetData();
	  //failed checksum, need to clear this out anyway
		rx_len = 0;
		rx_array_inx = 0;
		return false;
	  }
        
    }
  }
  return false;
}


		EasyTransfer3 messageOut;
		EasyTransfer3 messageIn;

void motorsStop(){
	messageOut.writeByte(COMMAND_MOTORS_STOP);
	messageOut.sendData();
}
void motorsWrite(int speedLeft,int speedRight){
	messageOut.writeByte(COMMAND_RUN);
	messageOut.writeInt(speedLeft);
	messageOut.writeInt(speedRight);
	messageOut.sendData();
}


/********************************************************************************/



#define ROWS_PER_IMAGE 32
#define PIXELS_PER_ROW 32
#define BYTES_PER_ROW 4

#define IMAGE_STATUS_INITIAL 0
#define IMAGE_STATUS_REQUESTED 1
#define IMAGE_STATUS_READING 2
#define IMAGE_STATUS_DONE 3
#define IMAGE_STATUS_ERROR -1

volatile unsigned long a=0;
volatile unsigned long b=0;

volatile byte image[ROWS_PER_IMAGE][PIXELS_PER_ROW];
// to reduce memory access operations during data reading (exclude array pointer increment) 
volatile byte pixel_00 = 0;
volatile byte pixel_01 = 0;
volatile byte pixel_02 = 0;
volatile byte pixel_03 = 0;
volatile byte pixel_04 = 0;
volatile byte pixel_05 = 0;
volatile byte pixel_06 = 0;
volatile byte pixel_07 = 0;
volatile byte pixel_08 = 0;
volatile byte pixel_09 = 0;
volatile byte pixel_10 = 0;
volatile byte pixel_11 = 0;
volatile byte pixel_12 = 0;
volatile byte pixel_13 = 0;
volatile byte pixel_14 = 0;
volatile byte pixel_15 = 0;
volatile byte pixel_16 = 0;
volatile byte pixel_17 = 0;
volatile byte pixel_18 = 0;
volatile byte pixel_19 = 0;
volatile byte pixel_20 = 0;
volatile byte pixel_21 = 0;
volatile byte pixel_22 = 0;
volatile byte pixel_23 = 0;
volatile byte pixel_24 = 0;
volatile byte pixel_25 = 0;
volatile byte pixel_26 = 0;
volatile byte pixel_27 = 0;
volatile byte pixel_28 = 0;
volatile byte pixel_29 = 0;
volatile byte pixel_30 = 0;
volatile byte pixel_31 = 0;
volatile byte curRow;
volatile byte curPixel;
volatile int curLine;
//volatile byte curOctet;
volatile int imageReadingStatus = IMAGE_STATUS_INITIAL;
int i=0;
int j=0;
int k=0;

#define INTERRUPT_PORT_FRAME 0
#define INTERRUPT_PORT_LINE 1
//#define PORT_FRAME 3
//#define PORT_LINE 2
#define PORT_PIXEL 22
#define PORT_LINE_ENABLE 21
//#define PORT_PRESCAILER_RESET 0
#define PORT_SERVO 6

#include <Servo.h> 

Servo weapon;

#define WEAPON_NORMAL_POSITION 20
#define WEAPON_KICK_POSITION 115

boolean doKick=false;

//#define MYDEBUG

void setup(){
//  pinMode(PORT_PRESCAILER_RESET,OUTPUT);
//  Robot.begin();
	Serial1.begin(9600);
	messageOut.begin(&Serial1);
	messageIn.begin(&Serial1);

//  Robot.setMode(MODE_SIMPLE);
	messageOut.writeByte(COMMAND_SWITCH_MODE);
	messageOut.writeByte(MODE_SIMPLE);
	messageOut.sendData();

  pinMode(PORT_LINE_ENABLE,OUTPUT);
  digitalRead(PORT_PIXEL);
  digitalRead(2);
  digitalRead(3);
  attachInterrupt(INTERRUPT_PORT_FRAME,onFrame,FALLING);
  attachInterrupt(INTERRUPT_PORT_LINE,onLine,FALLING);

  weapon.attach(PORT_SERVO);
  weapon.write(WEAPON_NORMAL_POSITION);
/*
  doKick=true;
  kickCockroach();
  doKick=true;
  kickCockroach();
  doKick=true;
  kickCockroach();
  */
#ifdef MYDEBUG
  Serial.begin(115200);
#endif
}

unsigned long prevShown = 0;
boolean hasShown = false;

int lSpeed;
int rSpeed;

#define CORRECTION_ADD 5//27

void loop(){
  
#ifdef MYDEBUG
  if(millis()-prevShown>500){
#else
  if(millis()-prevShown>50){
#endif
    requestImage();
    while(!isImageReady()){
      delayMicroseconds(200);
    }
    calcSpeed_Cockroach();
#ifdef MYDEBUG
    showImage();
#endif
    motorsWrite(rSpeed+CORRECTION_ADD,lSpeed-CORRECTION_ADD); //Make the robot go forward, full speed
    kickCockroach();
    prevShown=millis();
  }
//  delay(300);
//    motorsWrite(100,100); //Make the robot go forward, full speed
  
//test();
}
float avg[ROWS_PER_IMAGE];
float exc;
float prevExc;
float diff;
float regulator;
byte counts[ROWS_PER_IMAGE];
int mass;
float distance;

unsigned long prevExcMicros = 0;
unsigned long excMicros = 0;

double curAvg=0;

#define MIDDLE_SPEED 140
#define SPEED_DIFF 130.0

#define FACTOR_PROPORT 0.8
#define FACTOR_DIFF 3.0

#define DIFF_REGULATOR_LIMIT 1.0

#define MASS_IGNORE 30
#define MASS_AVOID 200

#define DISTANCE_PROPORT 15

#define TARGET_DISTANCE 15
#define AVOID_DISTANCE 22

#define TARGET_DEVIATION 5

void calcSpeed_Cockroach() {
  int curCount;
  //compute averages for all rows
  for(i=2;i<ROWS_PER_IMAGE-1;i++){
    curAvg=0;
    curCount=0;
    for(j=1;j<PIXELS_PER_ROW-1;j++){
        if((image[i][j] & 2) <= 0) {
          curAvg=curAvg+j;
          curCount=curCount+1;
        }
    }
    if(curCount>0){
      curAvg=curAvg/curCount-(PIXELS_PER_ROW-2)/2;
    }
    else {
      curCount=0;
    }
    avg[i]=curAvg;
    counts[i]=curCount;
  }
  //compute average values for whole image
  curAvg=0;
  curCount=0;
  mass=0;
  distance=0.0;
  for(i=3;i<ROWS_PER_IMAGE-1;i++){
    if(counts[i]>0){
      curAvg=curAvg+avg[i];
      curCount++;
      mass=mass+counts[i];
      distance=distance+i*counts[i];
    }
  }
  //horisontal excenticitet
  if(curCount>0){
    curAvg=curAvg/curCount;
  }
  else{
    curAvg=0.0;
  }
  exc=2*(curAvg)/(PIXELS_PER_ROW-2);
  excMicros=micros();
  if(prevExcMicros>0){
    diff=(float)10000.0*(exc-prevExc)/(excMicros-prevExcMicros);
  }
  else{
    diff=0;
  }
  prevExcMicros=excMicros;
  regulator=FACTOR_PROPORT*exc+constrain((float)diff*(float)FACTOR_DIFF,-DIFF_REGULATOR_LIMIT,DIFF_REGULATOR_LIMIT);
  //distance
  if(mass>0){
    distance=distance/mass;
  }
  else{
    distance=0.0;
  }
  distance=ROWS_PER_IMAGE-distance;
  //depending on mass, do action
  if(mass<MASS_IGNORE) { //if too less black pixels, ignore (stop)
    lSpeed=0;
    rSpeed=0;
  }
  else {
    if(mass<MASS_AVOID) { // if do not ignore and not avoid (attack)
      if((abs(distance-TARGET_DISTANCE)<=TARGET_DEVIATION) && (abs(curAvg)<=TARGET_DEVIATION)){
        lSpeed=0;
        rSpeed=0;
        doKick=true;
      }
      else {
        lSpeed=constrain(round(DISTANCE_PROPORT*(distance-TARGET_DISTANCE))+round(SPEED_DIFF*regulator),-255,255);
        rSpeed=constrain(round(DISTANCE_PROPORT*(distance-TARGET_DISTANCE))-round(SPEED_DIFF*regulator),-255,255);
      }
    }
    else { // avoid!
      lSpeed=constrain(round(DISTANCE_PROPORT*(distance-AVOID_DISTANCE))+round(SPEED_DIFF*regulator),-255,255);
      rSpeed=constrain(round(DISTANCE_PROPORT*(distance-AVOID_DISTANCE))-round(SPEED_DIFF*regulator),-255,255);
    }
  }
}

void kickCockroach(){
  if(doKick){
    weapon.write(WEAPON_KICK_POSITION);
    delay(600);
    weapon.write(WEAPON_NORMAL_POSITION);
    delay(450);
    doKick=false;
  }
}
  
#ifdef MYDEBUG
void showImage() {
  char imgRow[PIXELS_PER_ROW+1];
  Serial.println(curLine);
  Serial.println("--------------------------------");
  for(i=0;i<ROWS_PER_IMAGE;i++){
    for(j=0;j<PIXELS_PER_ROW;j++){
      if(image[i][j]==0) {
        imgRow[j]='.';
      }
      else {
        if((image[i][j] & 2) > 0) {
          imgRow[j]=' ';
        }
        else{
          imgRow[j]='#';
        }
      }
    }
    imgRow[PIXELS_PER_ROW]='\0';
    Serial.print(i);
    Serial.print('\t');
    Serial.print(imgRow);
    Serial.print('\t');
    Serial.println(avg[i]);
  }
  Serial.println("");
  Serial.print(exc);
  Serial.print('\t');
  Serial.print(diff);
  Serial.print('\t');
  Serial.print(regulator);
  Serial.print('\t');
  Serial.print(mass);
  Serial.print('\t');
  Serial.print(distance);
  Serial.print('\t');
  Serial.print(curAvg);
  Serial.print('\t');
  Serial.print(lSpeed);
  Serial.print('\t');
  Serial.println(rSpeed);
  Serial.println("--------------------------------");
}
#endif

void requestImage(){
  for(i=0;i<ROWS_PER_IMAGE;i++){
    for(j=0;j<PIXELS_PER_ROW;j++){
      image[i][j]=0;
    }
  }
/*
  digitalWrite(PORT_PRESCAILER_RESET,HIGH);
  asm("NOP");
  digitalWrite(PORT_PRESCAILER_RESET,LOW);
*/
  imageReadingStatus = IMAGE_STATUS_REQUESTED;
}

boolean isImageReady(){
  if(imageReadingStatus == IMAGE_STATUS_DONE) {
    return true;
  }
  else {
    return false;
  }
}

void stopReading(){
  digitalWrite(PORT_LINE_ENABLE,LOW);
  imageReadingStatus = IMAGE_STATUS_DONE;
}

void onFrame() {
// TODO не забыть отпрвлять импульс на reset предделителя!!!!!!!!!  
  if((imageReadingStatus == IMAGE_STATUS_REQUESTED)/* || (imageReadingStatus == IMAGE_STATUS_READING)*/){
    curRow=0;
    curLine=0;
    curPixel=0;  
    imageReadingStatus = IMAGE_STATUS_READING;
    digitalWrite(PORT_LINE_ENABLE,HIGH);
  }
}

#define _delay_between_pixels asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
#define _delay_between_pixels_2 _delay_between_pixels asm("nop");

void onLine() {  
  curRow++;
  curLine++;
  if(curRow<ROWS_PER_IMAGE){
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    
    pixel_00=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_01=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_02=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_03=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_04=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_05=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_06=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_07=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_08=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_09=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_10=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_11=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_12=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_13=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_14=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_15=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_16=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_17=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_18=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_19=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_20=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_21=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_22=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_23=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_24=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_25=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_26=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_27=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_28=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_29=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_30=*portInputRegister(6);
    _delay_between_pixels_2
    pixel_31=*portInputRegister(6);

    image[curRow][0] = pixel_00;
    image[curRow][1] = pixel_01;
    image[curRow][2] = pixel_02;
    image[curRow][3] = pixel_03;
    image[curRow][4] = pixel_04;
    image[curRow][5] = pixel_05;
    image[curRow][6] = pixel_06;
    image[curRow][7] = pixel_07;
    image[curRow][8] = pixel_08;
    image[curRow][9] = pixel_09;
    image[curRow][10] = pixel_10;
    image[curRow][11] = pixel_11;
    image[curRow][12] = pixel_12;
    image[curRow][13] = pixel_13;
    image[curRow][14] = pixel_14;
    image[curRow][15] = pixel_15;
    image[curRow][16] = pixel_16;
    image[curRow][17] = pixel_17;
    image[curRow][18] = pixel_18;
    image[curRow][19] = pixel_19;
    image[curRow][20] = pixel_20;
    image[curRow][21] = pixel_21;
    image[curRow][22] = pixel_22;
    image[curRow][23] = pixel_23;
    image[curRow][24] = pixel_24;
    image[curRow][25] = pixel_25;
    image[curRow][26] = pixel_26;
    image[curRow][27] = pixel_27;
    image[curRow][28] = pixel_28;
    image[curRow][29] = pixel_29;
    image[curRow][30] = pixel_30;
    image[curRow][31] = pixel_31;
  }
  else {
    stopReading();
  }
}

void onFrame1() { 
 a++; 
}

void onLine1() { 
 b++; 
}


