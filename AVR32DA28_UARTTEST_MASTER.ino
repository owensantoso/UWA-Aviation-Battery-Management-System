#include <SPI.h>


// Serial Comms
#define TX    4
#define RX    5

// BQ comms pins
#define MOSI1 8
#define MISO1 9
#define SCLK1 10
#define CS1   11


// To use in SPI.swap() function to swap between communicating to BQ and to primary board
#define PI_SPI SPI0_SWAP_DEFAULT
#define BQ_SPI SPI1_SWAP_DEFAULT
// To use before and after communication
#define SPIBeginTrans  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
#define SPIEndTrans    SPI.endTransaction();

// LED pins
#define LED1  19
#define LED2  18
#define LED3  17
#define LED4  16

// Misc. pins
#define ALERT 12
#define DFETOFF 13
#define UPDI  19

// 28 pins of AVR are broken down as:
// -- Connected (15)
// -- VDD/AVDD (2)
// -- Ground (2) 
// -- Shorted/Grounded (9)






void setup() {
  // put your setup code here, to run once:
  Serial.pins(TX,RX); // tx rx
  Serial.begin(115200); 
  flushReceive();
  // LED output pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // Primary board comms SPI pins
  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT);

  // BQ SPI pins
  pinMode(MOSI1, OUTPUT);
  pinMode(MISO1, INPUT);
  pinMode(SCLK1, OUTPUT);
  pinMode(CS1, OUTPUT);

  // Chip Select (CS) should always be active (low)
  digitalWrite(CS1, LOW);
}

int count = 0;
uint8_t Test_Data [2] = {0xFF, 0xA0}; 
uint8_t rxdata = 0;

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("Hello world! ");
  //Serial.println(count);
  count++;
  Serial.println("Sending 1");
  Serial.write(1);
  digitalWrite(LED1,HIGH);
  delay(500);
  digitalWrite(LED1,LOW);
  //delay(1000);
  while(Serial.available() == 0){}
  if(Serial.available()>0){
    rxdata = Serial.read();
    flushReceive();
    //if(rxdata == 1){
      Serial.print("Receiving 1: ");
      Serial.println(rxdata, BIN);
      digitalWrite(LED1,HIGH);
      delay(100);
      digitalWrite(LED1,LOW);
      delay(1000);
    //}
    Serial.flush();
  }
  
  
  Serial.println("Sending 2");
  Serial.write(2);
  digitalWrite(LED3,HIGH);
  delay(500);
  digitalWrite(LED3,LOW);
  //delay(1000);
  
  while(Serial.available() == 0){}
  if(Serial.available()>0){
    rxdata = Serial.read();
    flushReceive();
    //if(rxdata == 2){ 
      Serial.print("Receiving 2: ");
      Serial.println(rxdata, BIN);
      digitalWrite(LED3,HIGH);
      delay(100);
      digitalWrite(LED3,LOW);
      delay(1000);
    //}
    Serial.flush();
  }       
}


void flushReceive(){
  while(Serial.available()){
    Serial.read();
  }
}

void blinkLED(int led, int timems, int blinknumber)
{
    for(int i=0;i<blinknumber;i++){
      digitalWrite(led, HIGH);
      delay(timems);
      digitalWrite(led, LOW);
      delay(timems);
    }
}




// -----------------------------------------------
// MORSE CODE
// -----------------------------------------------

void dot(int led){
  digitalWrite(led, HIGH); delay(250);
  digitalWrite(led, LOW); delay(250);  
}

void dash(int led){
  digitalWrite(led, HIGH); delay(750);
  digitalWrite(led, LOW); delay(250);
}

void morse1(int led){
  dot(led);dash(led);dash(led);dash(led);dash(led);
}
void morse2(int led){
  dot(led);dot(led);dash(led);dash(led);dash(led);
}
void morse3(int led){
  dot(led);dot(led);dot(led);dash(led);dash(led);
}
void morse4(int led){
  dot(led);dot(led);dot(led);dot(led);dash(led);
}
void morse5(int led){
  dot(led);dot(led);dot(led);dot(led);dot(led);
}
void morse6(int led){
  dash(led);dot(led);dot(led);dot(led);dot(led);
}
void morse7(int led){
  dash(led);dash(led);dot(led);dot(led);dot(led);
}
void morse8(int led){
  dash(led);dash(led);dash(led);dot(led);dot(led);
}
void morse9(int led){
  dash(led);dash(led);dash(led);dash(led);dot(led);
}
void morse0(int led){
  dash(led);dash(led);dash(led);dash(led);dash(led);
}

void morseNumber(int led, int number)
{
  switch(number){
    case 0:
    morse0(led);
    break;
    case 1:
    morse1(led);
    break;
    case 2:
    morse2(led);
    break;
    case 3:
    morse3(led);
    break;
    case 4:
    morse4(led);
    break;
    case 5:
    morse5(led);
    break;
    case 6:
    morse6(led);
    break;
    case 7:
    morse7(led);
    break;
    case 8:
    morse8(led);
    break;
    case 9:
    morse9(led);
    break;
  }
}
