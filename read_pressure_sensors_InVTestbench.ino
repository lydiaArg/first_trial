//-------------------------------------------------------------------------------------------------
//
// Reading pressure values from two MS5803-14ba pressure sensors
//
// Author: Sofia Ntella
// Date: 05.12.2019
//-------------------------------------------------------------------------------------------------

#include <SPI.h>

// MS5803-14ba commands
#define CMD_RESET 0x1E
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command
#define CMD_ADC_D1 0x00 // ADC D1 conversion
#define CMD_ADC_D2 0x10 // ADC D2 conversion
#define CMD_ADC_256 0x00 // ADC OSR=256
#define CMD_ADC_512 0x02 // ADC OSR=512
#define CMD_ADC_1024 0x04 // ADC OSR=1024
#define CMD_ADC_2048 0x06 // ADC OSR=2056
#define CMD_ADC_4096 0x08 // ADC OSR=4096
#define CMD_PROM_RD 0xA0 // Prom read command

// Definitions

byte address = 0x00;
int CS1 = 10;
int CS2 = 9;
int PS1 = 8;
int PS2 = 7;
unsigned long D1; // ADC value of the pressure conversion
unsigned long D2; // ADC value of the temperature conversion
unsigned int C[8]; // calibration coefficients
double P; // compensated pressure value
double T; // compensated temperature value
double dT; // difference between actual and measured temperature
double OFF; // offset at actual temperature
double SENS; // sensitivity at actual temperature
double T2; // compensated pressure value, 2nd order
double OFF2; // compensated pressure value, 2nd order
double SENS2; // compensated pressure value, 2nd order

SPISettings settings(20000000, MSBFIRST, SPI_MODE0);

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting communication with sensors...");
  
  pinMode (CS1, OUTPUT);
  pinMode (CS2, OUTPUT);
  pinMode (PS1, OUTPUT);
  digitalWrite(PS1,LOW); //set pressure sensor 1 to SPI mode
  pinMode (PS2, OUTPUT);
  digitalWrite(PS2,LOW); //set pressure sensor 2 to SPI mode

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  SPI.beginTransaction(settings);
  cmd_reset(); // reset the module after powerup
  for (i=0;i<8;i++){ C[i]=cmd_prom(i);} // read calibration coefficients
  //n_crc=crc4(C);
  SPI.endTransaction();
  
}

void loop()
{
  read_calculate_pressure(CS1);
  read_calculate_pressure(CS2);
  delay(1);
}


void cmd_reset(void)
{
  digitalWrite(CS1, LOW); // pull CSB low to start the command
  SPI.transfer(CMD_RESET); // send reset sequence
  delay(3); // wait for the reset sequence timing
  digitalWrite(CS1, HIGH); // pull CSB high to finish the command
}



unsigned int cmd_prom(char coef_num)
{
  unsigned int ret;
  unsigned int rC = 0;
 
  digitalWrite(CS1, LOW); //pull CSB low
  SPI.transfer(CMD_PROM_RD + coef_num * 2); // send PROM READ command
  ret = SPI.transfer(0x00); // send 0 to read the MSB
  rC = 256 * ret;
  ret = SPI.transfer(0x00); // send 0 to read the LSB
  rC = rC + ret;
  digitalWrite(CS1, HIGH); // pull CSB high
  return rC;
}



unsigned long cmd_adc(char cmd, int sensor_selection)
{
  digitalWrite(sensor_selection, LOW);
  unsigned int ret;
  unsigned long temp = 0;
  cmd = SPI.transfer(CMD_ADC_CONV + cmd); // send conversion command
  switch (cmd & 0x0f) // wait necessary conversion time
  {
    case CMD_ADC_256 : delayMicroseconds(650); break;
    case CMD_ADC_512 : delay(3); break;
    case CMD_ADC_1024: delay(4); break;
    case CMD_ADC_2048: delay(6); break;
    case CMD_ADC_4096: delay(10); break;
  }
  digitalWrite(sensor_selection, HIGH); // csb_hi(); // pull CSB high to finish the conversion
  digitalWrite(sensor_selection, LOW); // csb_lo(); // pull CSB low to start new command
  SPI.transfer(CMD_ADC_READ); // send ADC read command
  ret = SPI.transfer(0x00); // send 0 to read 1st byte (MSB)
  temp = 65536 * ret;
  ret = SPI.transfer(0x00); // send 0 to read 2nd byte
  temp = temp + 256 * ret;
  SPDR = SPI.transfer(0x00); // send 0 to read 3rd byte (LSB)
  ret = SPDR;
  temp = temp + ret;
  digitalWrite(sensor_selection, HIGH); // pull CSB high to finish the read command
  return temp;
}



void read_calculate_pressure(int sensor_selection)
{
  SPI.beginTransaction(settings);

  D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_256, sensor_selection); // read uncompensated pressure
  D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_256, sensor_selection); // read uncompensated temperature
 
  // calcualte 1st order temperature (MS5803_14ba 1st order algorithm), base for 2nd order temperature and pressure
  dT = D2 - C[5] * pow(2, 8);
  OFF = C[2] * pow(2, 16) + dT * C[4] / pow(2, 7);
  SENS = C[1] * pow(2, 15) + dT * C[3] / pow(2, 8);
 
  T = (2000 + (dT * C[6]) / pow(2, 23)) / 100;
 
  // calcualte 2nd order pressure and temperature (MS5803_01b 2nd order algorithm)
  if (T >= 20) {
    T2 = 7 * pow(dT, 2) / pow(2, 37);
    OFF2 = 1 * pow((100*T - 2000),2) / pow(2, 4);
    SENS2 = 0;
 
  }
  else {
    T2 = 3 * pow(dT, 2) / pow(2, 33);
    OFF2 = 3 * pow(100*T - 2000, 2) / 2;
    SENS2 = 5 * pow(100*T - 2000, 2) / pow(2, 3);
  }
 
  //Recalculate T, OFF, SENS based on T2, OFF2, SENS2
  T -= T2;
  OFF -= OFF2;
  SENS -= SENS2;
 
  P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;
  SPI.endTransaction();
  Serial.println(T);
  Serial.println(P);
  delay(1);
}
