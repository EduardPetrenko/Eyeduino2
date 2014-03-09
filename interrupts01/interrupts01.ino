/********************************************************************************
Eduard Petrenko, Feb 2014
Used for search of ponts to connect with external interrupt possibility on the Arduino Robot Control Board
********************************************************************************/


volatile int count_0 = 0;
volatile int count_1 = 0;
volatile int count_2 = 0;
volatile int count_3 = 0;
volatile int count_4 = 0;

void setup(){
  attachInterrupt(0, _count0, CHANGE);
  attachInterrupt(1, _count1, CHANGE);
  attachInterrupt(2, _count2, CHANGE);
  attachInterrupt(3, _count3, CHANGE);
  attachInterrupt(4, _count4, CHANGE);
  Serial.begin(115200);
}

void loop(){
  Serial.print(count_0);
  Serial.print("\t");
  Serial.print(count_1);
  Serial.print("\t");
  Serial.print(count_2);
  Serial.print("\t");
  Serial.print(count_3);
  Serial.print("\t");
  Serial.println(count_4);
  delay(200);
}

void _count0(){
  count_0++;
}

void _count1(){
  count_1++;
}

void _count2(){
  count_2++;
}

void _count3(){
  count_3++;
}

void _count4(){
  count_4++;
}


