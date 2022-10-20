#include <SPI.h>


// Primary board comms pins
#define MOSI0 2
#define MISO0 3
#define SCLK0 4
#define CS0   5

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






/* Includes ------------------------------------------------------------------*/


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "BQ769x2Header.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEV_ADDR  0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 0  // 0 for disabled, 1 for enabled
#define MAX_BUFFER_SIZE 10
#define R 0 // Read; Used in DirectCommands and Subcommands functions
#define W 1 // Write; Used in DirectCommands and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
uint8_t CRC_check;
uint8_t rxdata [2];
uint8_t RX_data [2] = {0x00, 0x00}; // used in several functions to store data read from BQ769x2
uint8_t RX_32Byte [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  //used in Subcommands read function
// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage [16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
float Temperature [3] = {0,0,0};
uint16_t Stack_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
uint16_t Pack_Current = 0x00;

uint16_t AlarmBits = 0x00;
uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;   // Permanent Fail Status Register A
uint8_t value_PFStatusB;   // Permanent Fail Status Register B
uint8_t value_PFStatusC;   // Permanent Fail Status Register C
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
uint16_t CB_ActiveCells;  // Cell Balancing Active Cells

uint8_t UV_Fault = 0;   // under-voltage fault state
uint8_t OV_Fault = 0;   // over-voltage fault state
uint8_t SCD_Fault = 0;  // short-circuit fault state
uint8_t OCD_Fault = 0;  // over-current fault state
uint8_t ProtectionsTriggered = 0; // Set to 1 if any protection triggers

uint8_t LD_ON = 0;  // Load Detect status bit
uint8_t DSG = 0;   // discharge FET state
uint8_t CHG = 0;   // charge FET state
uint8_t PCHG = 0;  // pre-charge FET state
uint8_t PDSG = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int; // in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Frac;// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Time;// in BQ769x2_READPASSQ func
/* USER CODE END PV */













void setup() {
  // put your setup code here, to run once:
  //Serial.pins(4,5); // tx rx
  //Serial.begin(115200); 
  SPI.swap(BQ_SPI);
  SPI.begin();
  //SPI.beginTransaction(SPISettings(24000000, LSBFIRST, SPI_MODE0)); // possibly unnecessary ?
  //SPI.setBitOrder(LSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV2);
  //SPI.setDataMode(SPI_MODE0);
  
  // LED output pins
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // Primary board comms SPI pins
  pinMode(MOSI0, OUTPUT);
  pinMode(MISO0, INPUT);
  pinMode(SCLK0, OUTPUT);
  pinMode(CS0, OUTPUT);

  // BQ SPI pins
  pinMode(MOSI1, OUTPUT);
  pinMode(MISO1, INPUT);
  pinMode(SCLK1, OUTPUT);
  pinMode(CS1, OUTPUT);

  // Chip Select (CS) should always be active (low)
  digitalWrite(CS0, LOW);
  digitalWrite(CS1, HIGH);
}

int count = 0;
uint8_t Test_Data [2] = {0xFF, 0xA0}; 

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Hello world! ");
  Serial.println(count);
  count++;
  digitalWrite(CS1, HIGH);
  
    
  //SPI.pins(MOSI1, MISO1, SCLK1, CS1);
  //blinkLED(LED1, 50, 10);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  for(int i = 0; i < 6; i++){
    digitalWrite(CS1, LOW);
    SPI.transfer(0x68);
    SPI.transfer(0xFF);
    SPI.transfer(0x94);
    digitalWrite(CS1, HIGH);
    delayMicroseconds(500);
    //delayMicroseconds(100);
  }
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE1));
  for(int i = 0; i < 6; i++){
    digitalWrite(CS1, LOW);
    SPI.transfer(0x68);
    SPI.transfer(0xFF);
    SPI.transfer(0x94);
    digitalWrite(CS1, HIGH);
    delayMicroseconds(500);
    //delayMicroseconds(100);
  }
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE2));
  for(int i = 0; i < 6; i++){
    digitalWrite(CS1, LOW);
    SPI.transfer(0x68);
    SPI.transfer(0xFF);
    SPI.transfer(0x94);
    digitalWrite(CS1, HIGH);
    delayMicroseconds(500);
    //delayMicroseconds(100);
  }
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE3));
  for(int i = 0; i < 6; i++){
    digitalWrite(CS1, LOW);
    SPI.transfer(0x68);
    SPI.transfer(0xFF);
    SPI.transfer(0x94);
    digitalWrite(CS1, HIGH);
    delayMicroseconds(500);
    //delayMicroseconds(100);
  }
  SPI.endTransaction();

//  for(int i = 0; i < 10; i++){
//    SPI.transfer(0x69);
//    SPI.transfer(0xFF);
//    SPI.transfer(0x1F);
//    delayMicroseconds(50);
//    //delayMicroseconds(100);
//  }
  blinkLED(LED4, 200, 2); // Blink 2x in to signal end of BQ SPI comm
  delay(2000);

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





unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.  
{
  unsigned char i;
  unsigned char checksum = 0;

  for(i=0; i<len; i++)
    checksum += ptr[i];

  checksum = 0xff & ~checksum;

  return(checksum);
}


unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions 
{
  unsigned char i;
  unsigned char crc=0;
  while(len--!=0)
  {
    for(i=0x80; i!=0; i/=2)
    {
      if((crc & 0x80) != 0)
      {
        crc *= 2;
        crc ^= 0x107;
      }
      else
        crc *= 2;

      if((*ptr & i)!=0)
        crc ^= 0x107;
    }
    ptr++;
  }
  return(crc);
}





// From example SPI code for STM32, adapted for AVR32DA28
void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{  //type: R = read, W = write
  uint8_t TX_data[2] = {0x00, 0x00};

  //little endian format
  TX_data[0] = data & 0xff;
  TX_data[1] = (data >> 8) & 0xff;

  if (type == R) {//Read
    SPI_ReadReg(command, RX_data, 2 ); //RX_data is a global variable
    delayMicroseconds(2000);
  }
  if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
    SPI_WriteReg(command,TX_data,2);
    delayMicroseconds(2000);
  }
}


void SPI_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
  // SPI Read. Includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
  // NEED TO ADD CRC CHECK
  uint8_t addr; 
  uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned int i;
  unsigned int match;
  unsigned int retries = 10;
    
  match = 0;
  addr = reg_addr;
    
  for(i=0; i<count; i++) {
    TX_Buffer[0] = addr;
    TX_Buffer[1] = 0xFF;
    TX_Buffer[2] = 0xFF;
    
    rxdata[0] = SPI.transfer(TX_Buffer[0]);
    delayMicroseconds(50);
    rxdata[1] = SPI.transfer(TX_Buffer[1]);
    delayMicroseconds(50);
    CRC_check = SPI.transfer(TX_Buffer[2]);
    delayMicroseconds(50);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    //HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); 
        
    while ((match == 0) & (retries > 0)) {
      delayMicroseconds(500);

      rxdata[0] = SPI.transfer(TX_Buffer[0]);
      delayMicroseconds(50);
      rxdata[1] = SPI.transfer(TX_Buffer[1]);
      delayMicroseconds(50);
      CRC_check = SPI.transfer(TX_Buffer[2]);
      delayMicroseconds(50);
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      //HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); 
      
      // Check for matching command (Slave sent command back meaning)
      // Also check for CRC8
      if ( (rxdata[0] == addr) & (CRC8(rxdata,2) == CRC_check) ) {
        match = 1;
        reg_data[i] = rxdata[1];
      }
      retries --;
    }    
    match = 0;
    addr += 1;
    delayMicroseconds(500);
  }
}


void SPI_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
  // SPI Write. Includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
  uint8_t addr; 
  uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned int i;
  unsigned int match;
  unsigned int retries = 10;
    
  match = 0;
  addr = 0x80 | reg_addr;
    
  for(i=0; i<count; i++) {
    TX_Buffer[0] = addr;
    TX_Buffer[1] = reg_data[i];
    TX_Buffer[2] = 0xFF;
    
    rxdata[0] = SPI.transfer(TX_Buffer[0]);
    rxdata[1] = SPI.transfer(TX_Buffer[1]);
    CRC_check = SPI.transfer(TX_Buffer[2]);
        
    while ((match == 0) & (retries > 0)) {
      delayMicroseconds(50);
      rxdata[0] = SPI.transfer(TX_Buffer[0]);
      rxdata[1] = SPI.transfer(TX_Buffer[1]);
      CRC_check = SPI.transfer(TX_Buffer[2]);
      // If BQ understood and executed the command, it should return the same as TX
      if ( (rxdata[0] == addr) & (rxdata[1] == reg_data[i]) & (CRC8(rxdata,2) == CRC_check) ){
        match = 1;
      }
      retries --;
    }    
    match = 0;
    addr += 1;
    delayMicroseconds(50);
  }
}


void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
  uint8_t TX_Buffer[2] = {0x00, 0x00};
  uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  //TX_RegData in little endian format
  TX_RegData[0] = reg_addr & 0xff; 
  TX_RegData[1] = (reg_addr >> 8) & 0xff;
  TX_RegData[2] = reg_data & 0xff; //1st byte of data

  switch(datalen)
    {
    case 1: //1 byte datalength
          SPI_WriteReg(0x3E, TX_RegData, 3);
      delayMicroseconds(2000);
      TX_Buffer[0] = Checksum(TX_RegData, 3); 
      TX_Buffer[1] = 0x05; //combined length of register address and data
          SPI_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      delayMicroseconds(2000);
      break;
    case 2: //2 byte datalength
      TX_RegData[3] = (reg_data >> 8) & 0xff;
      SPI_WriteReg(0x3E, TX_RegData, 4);
      delayMicroseconds(2000);
      TX_Buffer[0] = Checksum(TX_RegData, 4); 
      TX_Buffer[1] = 0x06; //combined length of register address and data
          SPI_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      delayMicroseconds(2000);
      break;
    case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
      TX_RegData[3] = (reg_data >> 8) & 0xff;
      TX_RegData[4] = (reg_data >> 16) & 0xff;
      TX_RegData[5] = (reg_data >> 24) & 0xff;
      SPI_WriteReg(0x3E, TX_RegData, 6);
      delayMicroseconds(2000);
      TX_Buffer[0] = Checksum(TX_RegData, 6); 
      TX_Buffer[1] = 0x08; //combined length of register address and data
          SPI_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      delayMicroseconds(2000);
      break;
    }
}


uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
  //RX_data is global var
  DirectCommands(command, 0x00, R);
  if(command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
    return (RX_data[1]*256 + RX_data[0]); //voltage is reported in mV
  }
  else {//stack, Pack, LD
    return 10 * (RX_data[1]*256 + RX_data[0]); //voltage is reported in 0.01V units
  }
}


uint16_t BQ769x2_ReadCurrent() 
// Reads PACK current 
{
  DirectCommands(CC2Current, 0x00, R);
  return (RX_data[1]*256 + RX_data[0]);  // current is reported in mA
}


float BQ769x2_ReadTemperature(uint8_t command) 
{
  DirectCommands(command, 0x00, R);
  //RX_data is a global var
  return (0.1 * (float)(RX_data[1]*256 + RX_data[0])) - 273.15;  // converts from 0.1K to Celcius
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
