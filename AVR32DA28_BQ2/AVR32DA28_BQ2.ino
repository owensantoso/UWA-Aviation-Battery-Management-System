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
uint8_t CRC_check_send;
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

int count = 0;











void setup() {
  // put your setup code here, to run once:
  Serial.pins(4,5); // tx rx -> MISO0 MOSI0 of board, pins 4 and 5
  Serial.begin(115200);
  SPI.swap(BQ_SPI);
  SPI.begin();
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
  digitalWrite(CS1, LOW);
  delay(2000);
  blinkLED(LED1, 50, 30);

  CommandSubcommands(SET_CFGUPDATE);
  //BQ769x2_SetRegister(SPIConfiguration, 0b01100000, 1); // set MISO to use REG1

  // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
  //BQ769x2_SetRegister(REG0Config, 0x01, 1);
  
  // 'REG12 Config' - Enable REG1 with 5V output (0x0D for 3.3V, 0x0F=0b00001111 for 5V)
  //BQ769x2_SetRegister(REG12Config, 0b00001111, 1); // REG1 change to 5V and enable 0000 1111

  // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
  // Temporarily disable - 0x92FD = 0x00
  BQ769x2_SetRegister(TS1Config, 0x00, 1);

  // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
  // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
  BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);
  
  // Set up ALERT Pin - 0x92FC = 0x2A
  // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
  // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
  BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);

  // 5th bit: 1 = Default host FET control state forces FETs off
  //BQ769x2_SetRegister(FETOptions, 0b00011101, 1);
  
  
  // 'VCell Mode' - Enable 6 cells 0000000010 10101011
  BQ769x2_SetRegister(VCellMode, 0b000000001010101011, 2);
  CommandSubcommands(EXIT_CFGUPDATE);
}


void loop() {
  // put your main code here, to run repeatedly:
  uint16_t VoltageTest = 9999;
  float InternalTemp = 9999;
  
  //SPI.pins(MOSI0, MISO0, SCLK0, CS0);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  
  blinkLED(LED4, 100, 5); // Blink 5x in 1 second to signal start of BQ SPI comm
  BQ769x2_ReadAllVoltages();
  Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
  Serial.print("Stack Voltage: ");
  Serial.print(Stack_Voltage);
  Serial.println("mV");
  Serial.print("Pack Voltage: ");
  Serial.print(Pack_Voltage);
  Serial.println("mV");
  Serial.print("LD Voltage: ");
  Serial.print(LD_Voltage);
  Serial.println("mV");


  // Print Cell Voltages 
  Serial.print("Cell ");
  Serial.print(1);
  Serial.print(" Voltage: ");
  Serial.print(CellVoltage[0]);
  Serial.println("mV");
    
  Serial.print("Cell ");
  Serial.print(2);
  Serial.print(" Voltage: ");
  Serial.print(CellVoltage[1]);
  Serial.println("mV");
  
  Serial.print("Cell ");
  Serial.print(3);
  Serial.print(" Voltage: ");
  Serial.print(CellVoltage[3]);
  Serial.println("mV");
  
  Serial.print("Cell ");
  Serial.print(4);
  Serial.print(" Voltage: ");
  Serial.print(CellVoltage[5]);
  Serial.println("mV");
  
  Serial.print("Cell ");
  Serial.print(5);
  Serial.print(" Voltage: ");
  Serial.print(CellVoltage[7]);
  Serial.println("mV");
  
  Serial.print("Cell ");
  Serial.print(6);
  Serial.print(" Voltage: ");
  Serial.print(CellVoltage[9]);
  Serial.println("mV");
  
//  for(int i = 0; i < 15; i++){
//    Serial.print("Cell ");
//    Serial.print(i+1);
//    Serial.print(" Voltage: ");
//    Serial.print(CellVoltage[i]);
//    Serial.println("mV");
//  }


//  Serial.print("TS1 Temperature: ");
//  Serial.print(Temperature[0]);
//  Serial.println("C");

  VoltageTest = BQ769x2_ReadVoltage(Cell1Voltage); // Cell voltage given in mV
  Serial.print("Cell 1 Voltage: ");
  Serial.print(VoltageTest);
  Serial.println("mV");
  InternalTemp = BQ769x2_ReadTemperature(IntTemperature); // Internal Temp in Celsius
  Serial.print("InternalTemp: ");
  Serial.print(InternalTemp);
  Serial.println("C");
  
  if(count%10==0 && count%20!=0){
    BQ769x2_BOTHOFF ();
  }
  if(count%20==0){
    BQ769x2_RESET_BOTHOFF ();
  }
  BQ769x2_ReadFETStatus();
  Serial.print("DSG: ");
  Serial.println(DSG);
  Serial.print("CHG: ");
  Serial.println(CHG);
  Serial.print("PCHG: ");
  Serial.println(PCHG);
  Serial.print("PDSG: ");
  Serial.println(PDSG);
  
  
  SPI.endTransaction();
  blinkLED(LED4,500,1);
  delay(200);  
  count++;
  /*
  SPI.swap(PI_SPI);
  SPI.begin();
  digitalWrite(LED2, HIGH);
  SPI.end();
  delay(2000);
  */

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

void CommandSubcommands(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{  //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively
  
  uint8_t TX_Reg[2] = {0x00, 0x00};

  //TX_Reg in little endian format
  TX_Reg[0] = command & 0xff;
  TX_Reg[1] = (command >> 8) & 0xff;

  SPI_WriteReg(0x3E,TX_Reg,2); 
  delayMicroseconds(2000);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
  //security keys and Manu_data writes dont work with this function (reading these commands works)
  //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
  uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t TX_Buffer[2] = {0x00, 0x00};

  //TX_Reg in little endian format
  TX_Reg[0] = command & 0xff;
  TX_Reg[1] = (command >> 8) & 0xff; 

  if (type == R) {//read
    SPI_WriteReg(0x3E,TX_Reg,2);
    delayMicroseconds(2000);
    SPI_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
  }
  else if (type == W) {
    //FET_Control, REG12_Control
    TX_Reg[2] = data & 0xff; 
    SPI_WriteReg(0x3E,TX_Reg,3);
    delayMicroseconds(1000);
    TX_Buffer[0] = Checksum(TX_Reg, 3);
    TX_Buffer[1] = 0x05; //combined length of registers address and data
    SPI_WriteReg(0x60, TX_Buffer, 2);
    delayMicroseconds(1000); 
  }
  else if (type == W2){ //write data with 2 bytes
    //CB_Active_Cells, CB_SET_LVL
    TX_Reg[2] = data & 0xff; 
    TX_Reg[3] = (data >> 8) & 0xff;
    SPI_WriteReg(0x3E,TX_Reg,4);
    delayMicroseconds(1000);
    TX_Buffer[0] = Checksum(TX_Reg, 4); 
    TX_Buffer[1] = 0x06; //combined length of registers address and data
    SPI_WriteReg(0x60, TX_Buffer, 2);
    delayMicroseconds(1000); 
  }
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
  unsigned int retries = 50;
    
  match = 0;
  addr = reg_addr;
    
  for(i=0; i<count; i++) {
    TX_Buffer[0] = addr;
    TX_Buffer[1] = 0xFF;
    CRC_check_send = CRC8(TX_Buffer,2);
//    Serial.print("TX_Buffer[0]: ");
//    Serial.println(TX_Buffer[0], HEX);
//    Serial.print("TX_Buffer[1]: ");
//    Serial.println(TX_Buffer[1], HEX);
//    Serial.print("TX_Buffer[2]: ");
//    Serial.println(TX_Buffer[2], HEX);
    
    digitalWrite(CS1, LOW);
    rxdata[0] = SPI.transfer(TX_Buffer[0]);
    delayMicroseconds(2);
    rxdata[1] = SPI.transfer(TX_Buffer[1]);
    delayMicroseconds(2);
    CRC_check = SPI.transfer(CRC_check_send);
    digitalWrite(CS1, HIGH);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    //HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); 
        
    while ((match == 0) & (retries > 0)) {
      delayMicroseconds(500);

      digitalWrite(CS1, LOW);
      rxdata[0] = SPI.transfer(TX_Buffer[0]);
      delayMicroseconds(2);
      rxdata[1] = SPI.transfer(TX_Buffer[1]);
      delayMicroseconds(2);
      CRC_check = SPI.transfer(CRC_check_send);
      digitalWrite(CS1, HIGH);
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//      HAL_SPI_TransmitReceive(&hspi1, TX_Buffer, rxdata, 2, 1);
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); 
      
      // Check for matching command (Slave sent command back meaning)
      // Also check for CRC8
      if ( (rxdata[0] == addr) & (CRC8(rxdata,2) == CRC_check) ) {
        match = 1;
        reg_data[i] = rxdata[1];
        //Serial.println("nice");
      }
      retries --;
    } 
//    Serial.print("rxdata[0]: ");
//    Serial.println(rxdata[0], HEX);
//    Serial.print("rxdata[1]: ");
//    Serial.println(rxdata[1], HEX);
//    Serial.print("CRC_check: ");
//    Serial.println(CRC_check, HEX);
//    
//    Serial.print("TX_Buffer[0]: ");
//    Serial.println(TX_Buffer[0], HEX);
//    Serial.print("TX_Buffer[1]: ");
//    Serial.println(TX_Buffer[1], HEX);
//    Serial.print("TX_Buffer[2]: ");
//    Serial.println(TX_Buffer[2], HEX);
//    Serial.println("---------------");
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
    CRC_check_send = CRC8(TX_Buffer,2);
    
    digitalWrite(CS1, LOW);
    rxdata[0] = SPI.transfer(TX_Buffer[0]);
    delayMicroseconds(2);
    rxdata[1] = SPI.transfer(TX_Buffer[1]);
    delayMicroseconds(2);
    CRC_check = SPI.transfer(CRC_check_send);
    digitalWrite(CS1, HIGH);
    while ((match == 0) & (retries > 0)) {
      delayMicroseconds(500);
      digitalWrite(CS1, LOW);
      rxdata[0] = SPI.transfer(TX_Buffer[0]);
      delayMicroseconds(2);
      rxdata[1] = SPI.transfer(TX_Buffer[1]);
      delayMicroseconds(2);
      CRC_check = SPI.transfer(CRC_check_send);
      digitalWrite(CS1, HIGH);
      // If BQ understood and executed the command, it should return the same as TX
      if ( (rxdata[0] == addr) & (rxdata[1] == reg_data[i]) & (CRC8(rxdata,2) == CRC_check) ){
        match = 1;
      }
      retries --;
    }    
    match = 0;
    addr += 1;
    delayMicroseconds(500);
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



void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
  int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
  for (int x = 0; x < 16; x++){//Reads all cell voltages
    CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
    cellvoltageholder = cellvoltageholder + 2;
  }
  Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
  Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
  LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
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


//  ********************************* FET Control Commands  ***************************************

void BQ769x2_BOTHOFF () {
  // Disables all FETs using the DFETOFF (BOTHOFF) pin
  // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
  digitalWrite(DFETOFF, HIGH);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // DFETOFF pin (BOTHOFF) set high
}

void BQ769x2_RESET_BOTHOFF () {
  // Resets DFETOFF (BOTHOFF) pin
  // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
  digitalWrite(DFETOFF, LOW);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // DFETOFF pin (BOTHOFF) set low
}

void BQ769x2_ReadFETStatus() { 
  // Read FET Status to see which FETs are enabled
  DirectCommands(FETStatus, 0x00, R);
  FET_Status = (RX_data[1]*256 + RX_data[0]);
  DSG = ((0x4 & RX_data[0])>>2);// discharge FET state
  CHG = (0x1 & RX_data[0]);// charge FET state
  PCHG = ((0x2 & RX_data[0])>>1);// pre-charge FET state
  PDSG = ((0x8 & RX_data[0])>>3);// pre-discharge FET state
}
