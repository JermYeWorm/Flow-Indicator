//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino pin connections:
//
//  |MAX30001 pin label| Pin Function         |Arduino Connection|
//  |----------------- |:--------------------:|-----------------:|
//  | MISO             | Slave Out            |  D12             |
//  | MOSI             | Slave In             |  D11             |
//  | SCLK             | Serial Clock         |  D13             |
//  | CS               | Chip Select          |  D10             |
//  | VCC              | Digital VDD          |  +5V             |
//  | GND              | Digital Gnd          |  Gnd             |
//  | FCLK             | 32K CLOCK            |  -               |
//  | INT1             | Interrupt1           |  02              |
//  | INT2             | Interrupt2           |  -               |
//
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include "MAX30001.h"

#define MAX30001_CS_PIN 10
#define MAX30001_DELAY_SAMPLES 8 // Time between consecutive samples

#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define CES_CMDIF_TYPE_DATA 0x02
#define CES_CMDIF_PKT_STOP 0x0B
#define DATA_LEN 0x0C
#define ZERO 0

volatile char DataPacket[DATA_LEN];
const char DataPacketFooter[2] = {ZERO, CES_CMDIF_PKT_STOP};
const char DataPacketHeader[5] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, DATA_LEN, ZERO, CES_CMDIF_TYPE_DATA};

uint8_t data_len = 0x0C;

MAX30001 max30001(MAX30001_CS_PIN);

signed long ecg_data;
signed long bioz_data;

void sendDataECG(signed long ecg_sample)
{
  /*  Data format (zero padding needed to perform signed conversion): 
   *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
   *  [                       DATA                        ] [ ETAG ] [ PTAG ] [       PADDING       ]
   */
  signed long ecg_buff = (signed long) (ecg_sample >> 8);
  signed long ecg_data = (signed long) (ecg_buff >> 6);
  uint8_t e_tag = (ecg_buff & 0x38) >> 3;                       // Align ETAG[2:0] to 8-bit val. 0b00XXX000 -> 0b00000XXX
  uint8_t p_tag = ecg_buff & 0x07;                              // PTAG[2:0] is already aligned. 0b00000XXX
  
  // Retrieve ECG gain config parameters
  max30001_cnfg_ecg_t cnfg_ecg;
  int ecg_gain = 10;
  if (cnfg_ecg.bit.gain == 0b00) {
    ecg_gain = 20;
  } else if (cnfg_ecg.bit.gain == 0b01) {
    ecg_gain = 40;
  } else if (cnfg_ecg.bit.gain == 0b10) {
    ecg_gain = 80;
  } else if (cnfg_ecg.bit.gain == 0b11) {
    ecg_gain = 160;
  }
  
  // Converting ECG samples to calibrated voltage values (eqn from datasheet)
  double ecg_data_cal = (ecg_data * 1000) / (pow(2, 17) * ecg_gain);  // Vref = 1000 mV by default

  Serial.print("ECG (mV): ");
  Serial.print(ecg_data_cal);
  // Serial.print(", ");
  // Serial.print("ETAG[2:0]: ");
  // Serial.println(e_tag);
  // Serial.print("PTAG[2:0]: ");
  // Serial.println(p_tag);
}

void sendDataBioZ(signed long bioz_sample)
{
  /*  Data format (zero padding needed to perform signed conversion): 
   *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
   *  [                          DATA                           ] [] [ BTAG ] [       PADDING       ]
   */
  signed long bioz_buff = (signed long) (bioz_sample >> 8);
  signed long bioz_data = (signed long) (bioz_buff >> 4);
  uint8_t b_tag = bioz_buff & 0x07; // BTAG[2:0] is already aligned. 0b00000XXX

  // Retrieve BIOZ gain & current magnitude config parameters
  max30001_cnfg_bioz_t cnfg_bioz;
  int bioz_gain = 10;
  if (cnfg_bioz.bit.gain == 0b00) {
    bioz_gain = 10;
  } else if (cnfg_bioz.bit.gain == 0b01) {
    bioz_gain = 20;
  } else if (cnfg_bioz.bit.gain == 0b10) {
    bioz_gain = 40;
  } else if (cnfg_bioz.bit.gain == 0b11) {
    bioz_gain = 80;
  }
  float cg_mag = 0.000032;
  if (cnfg_bioz.bit.cgmag == 0b000) {
    cg_mag = 0;
  } else if (cnfg_bioz.bit.gain == 0b001) {
    cg_mag = 0.00008;
  } else if (cnfg_bioz.bit.gain == 0b010) {
    cg_mag = 0.000016;
  } else if (cnfg_bioz.bit.gain == 0b011) {
    cg_mag = 0.000032;
  } else if (cnfg_bioz.bit.gain == 0b100) {
    cg_mag = 0.000048;
  } else if (cnfg_bioz.bit.gain == 0b101) {
    cg_mag = 0.000064;
  } else if (cnfg_bioz.bit.gain == 0b110) {
    cg_mag = 0.000080;
  } else if (cnfg_bioz.bit.gain == 0b111) {
    cg_mag = 0.000096;
  }

  // Converting BIOZ samples to calibrated resistance values (eqn from datasheet)
  double bioz_data_cal = (bioz_data * 1) / (pow(2, 19) * bioz_gain * cg_mag);   // Vref = 1 V by default

  Serial.print("EBI (Ohms): ");
  Serial.print(bioz_data_cal);

}

bool BioZSkipSample = false;

void setup()
{
  Serial.begin(57600); // Serial begin

  SPI.begin();

  bool ret = max30001.max30001ReadInfo();
  if (ret)
  {
    Serial.println("MAX30001 read ID Success");
  }
  else
  {
    while (!ret)
    {
      // Stay here until the issue is fixed.
      ret = max30001.max30001ReadInfo();
      Serial.println("Failed to read ID, please make sure all the pins are connected");
      delay(5000);
    }
  }

  Serial.println("Initialising the device ...");
  max30001.BeginConfig(); // initialize MAX30001
}

void loop()
{
  ecg_data = max30001.getECGSamples();
  sendDataECG(ecg_data);
  Serial.print(", ");

  bioz_data = max30001.getBioZSamples();
  sendDataBioZ(bioz_data);
  Serial.print("\n");

  int bioz_sps = 64;
  float bioz_delay = 1000 * (float)1/bioz_sps;

  delay(bioz_delay);
}