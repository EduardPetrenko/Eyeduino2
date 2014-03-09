/***********************************************************************************
February 2014, E.Petrenko
video load into Arduino (Eyeduino project)
Checks enable feature of counter/prescaler. After pressing button counter is disabled,stopping interrupts.

***********************************************************************************/


volatile unsigned long a=0;
volatile unsigned long b=0;
volatile unsigned long bStored=0;

#define INTERRUPT_PORT_FRAME 0
#define INTERRUPT_PORT_LINE 1

#define PORT_FRAME 3
#define PORT_LINE 2
#define PORT_LINE_ENABLE 21
#define PORT_BUTTON 22

void setup(){
  attachInterrupt(INTERRUPT_PORT_FRAME,onFrame1,FALLING);
  attachInterrupt(INTERRUPT_PORT_LINE,onLine1,FALLING);
  pinMode(PORT_LINE_ENABLE,OUTPUT);
  pinMode(PORT_BUTTON,INPUT_PULLUP);
  Serial.begin(115200);
}

void loop(){
 delay(1000);
 Serial.print(millis());
 Serial.print("\t");
 if(digitalRead(PORT_BUTTON)==LOW) {
   digitalWrite(PORT_LINE_ENABLE,LOW);
   Serial.print("_");   
 }
 else {
   digitalWrite(PORT_LINE_ENABLE,HIGH);
   Serial.print("T");   
 }
 Serial.print("\t");
 Serial.print(a);
 a=0;
 Serial.print("\t");
 Serial.println(bStored);
}

//this is called for each frame that acheives MCU (Arduino digital port 3)
void onFrame1() { 
 a++; 
 bStored=b;
 b=0;
}

//this is called for each line that acheives MCU (Arduino digital port 2)
void onLine1() { 
 b++; 
}

