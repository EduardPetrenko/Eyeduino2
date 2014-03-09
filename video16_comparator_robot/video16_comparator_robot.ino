/***********************************************************************************
February 2014, E.Petrenko
video load into Arduino (Eyeduino project)
Try to store value from comparator through digital port due to hardware pre-scaler line interrupt is invoked only for every tenth line of image.

Terms:
LINE is line of original image (if we use line filtering by Arduino program)
ROW is row of image that is stored into the memory
***********************************************************************************/


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


void setup(){
  attachInterrupt(INTERRUPT_PORT_FRAME,onFrame,FALLING);
  attachInterrupt(INTERRUPT_PORT_LINE,onLine,FALLING);
  pinMode(PORT_LINE_ENABLE,OUTPUT);
  digitalRead(PORT_PIXEL);
//  pinMode(PORT_PRESCAILER_RESET,OUTPUT);
  Serial.begin(115200);
  test();
}

unsigned long prevShown = 0;
boolean hasShown = false;

void loop(){
  if(millis()-prevShown>500){
    requestImage();
    while(!isImageReady()){
      delayMicroseconds(200);
    }
    showImage();
    prevShown=millis();
  }
//test();
}

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
    Serial.println(imgRow);
  }
  Serial.println("--------------------------------");
}

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

void test() {
  /*
  unsigned long prevMicros = micros();
  for(i=0;i<1000;i++) {
    pixel_00=*portInputRegister(4);
    pixel_01=*portInputRegister(4);
    pixel_02=*portInputRegister(4);
    pixel_03=*portInputRegister(4);
    pixel_04=*portInputRegister(4);
    pixel_05=*portInputRegister(4);
    pixel_06=*portInputRegister(4);
    pixel_07=*portInputRegister(4);
    pixel_08=*portInputRegister(4);
    pixel_09=*portInputRegister(4);
    pixel_10=*portInputRegister(4);
    pixel_11=*portInputRegister(4);
    pixel_12=*portInputRegister(4);
    pixel_13=*portInputRegister(4);
    pixel_14=*portInputRegister(4);
    pixel_15=*portInputRegister(4);
    pixel_16=*portInputRegister(4);
    pixel_17=*portInputRegister(4);
    pixel_18=*portInputRegister(4);
    pixel_19=*portInputRegister(4);
    pixel_20=*portInputRegister(4);
    pixel_21=*portInputRegister(4);
    pixel_22=*portInputRegister(4);
    pixel_23=*portInputRegister(4);
    pixel_24=*portInputRegister(4);
    pixel_25=*portInputRegister(4);
    pixel_26=*portInputRegister(4);
    pixel_27=*portInputRegister(4);
    pixel_28=*portInputRegister(4);
    pixel_29=*portInputRegister(4);
    pixel_30=*portInputRegister(4);
    pixel_31=*portInputRegister(4);
  }
  Serial.println(micros()-prevMicros);
  curRow=0;
  prevMicros = micros();
  for(i=0;i<1000;i++) {
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
  Serial.println(micros()-prevMicros); 
 */ 
//  Serial.println(int(portInputRegister(4)),HEX);
  Serial.println(digitalPinToTimer(PORT_PIXEL));
  Serial.println(digitalPinToBitMask(PORT_PIXEL));
  Serial.println(digitalPinToPort(PORT_PIXEL));
  Serial.println("----");
  delay(400);
}

