//
//  Arduino pin connections:
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

#define MAX30001_SPI_SPEED 1000000

MAX30001::MAX30001(int cs_pin)
{
    _cs_pin = cs_pin;
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    ecgSamplesAvailable = 0;
}

void MAX30001::_max30001RegWrite(unsigned char WRITE_ADDRESS, unsigned long data)
{
    // Combine the register address and the command into one byte:
    byte dataToSend = (WRITE_ADDRESS << 1) | WREG;

    SPI.beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    delay(2);
    SPI.transfer(dataToSend);
    SPI.transfer(data >> 16);
    SPI.transfer(data >> 8);
    SPI.transfer(data);
    delay(2);

    digitalWrite(_cs_pin, HIGH);

    SPI.endTransaction();
}

void MAX30001::_max30001RegRead24(uint8_t Reg_address, uint32_t *read_data)
{
    uint8_t spiTxBuff;

    uint8_t buff[4];

    SPI.beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (Reg_address << 1) | RREG;
    SPI.transfer(spiTxBuff); // Send register location

    for (int i = 0; i < 3; i++)
    {
        buff[i] = SPI.transfer(0xff);
    }

    digitalWrite(_cs_pin, HIGH);

    *read_data = (buff[0] << 16) | (buff[1] << 8) | buff[2];

    SPI.endTransaction();
}

void MAX30001::_max30001RegRead(uint8_t Reg_address, uint8_t *buff)
{
    uint8_t spiTxBuff;

    SPI.beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (Reg_address << 1) | RREG;
    SPI.transfer(spiTxBuff); // Send register location

    for (int i = 0; i < 3; i++)
    {
        buff[i] = SPI.transfer(0xff);
    }

    digitalWrite(_cs_pin, HIGH);

    SPI.endTransaction();
}

void MAX30001::_max30001SwReset(void)
{
    _max30001RegWrite(SW_RST, 0x000000);
    delay(100);
}

void MAX30001::_max30001Synch(void)
{
    _max30001RegWrite(SYNCH, 0x000000);
}

void MAX30001::_max30001FIFOReset(void)
{
    _max30001RegWrite(FIFO_RST, 0x000000);
}

bool MAX30001::max30001ReadInfo(void)
{
    uint8_t readBuff[4];

    _max30001RegRead(INFO, readBuff);

    if ((readBuff[0] & 0xf0) == 0x50)
    {
        Serial.print("MAX30001 Detected. Rev ID:  ");
        Serial.println((readBuff[0] & 0xf0));

        return true;
    }
    else
    {

        Serial.println("MAX30001 read info error\n");
        return false;
    }

    return false;
}

void MAX30001::BeginConfig()
{
    uint8_t regReadBuff[4];
    unsigned long status0 = (unsigned long)(regReadBuff[0]);
    status0 = status0 << 16;
    unsigned long status1 = (unsigned long)(regReadBuff[1]);
    status1 = status1 << 8;
    unsigned long status2 = (unsigned long)(regReadBuff[2]);
    unsigned long status = (unsigned long)(status0 | status1 | status2);
    unsigned long data_read;
    Serial.print("Status bits: ");
    for (int i = 23; i >= 0; i--) {
        // Shift right by i and mask to get the specific bit
        data_read = (status >> i) & 1;
        Serial.print(data_read);
    }
    Serial.print("\n\n");

    // BioZ Self-Test
    // _max30001RegWrite(CNFG_BMUX, 0x301C01);
    // delay(100);
    // _max30001RegWrite(CNFG_BIOZ, 0x601130);
    // delay(100);

    // BioZ (only) Acquisition Mode
    _max30001SwReset();
    delay(100);
    _max30001RegWrite(CNFG_GEN, 0x040027); // Resistive Lead Bias on BioZ Ch.
    delay(100);
    _max30001RegWrite(CNFG_CAL, 0x004800);
    delay(100);
    _max30001RegWrite(CNFG_EMUX, 0x300000);
    delay(100);
    _max30001RegWrite(CNFG_ECG, 0x805000); // d23 - d22 : 10 for 250sps , 00:500 sps
    delay(100);
    _max30001RegWrite(CNFG_RTOR1, 0x3F2300);
    delay(100);

    _max30001RegWrite(CNFG_BMUX, 0x001040);
    delay(100);
    _max30001RegWrite(CNFG_BIOZ, 0x640170);
    delay(100);

    _max30001Synch();
    delay(100);

  // BioZ & ECG (w/ filters)
    // _max30001SwReset();
    // delay(100);
    // _max30001RegWrite(CNFG_GEN, 0x0C0017); // Resistive Lead Bias on ECG Ch.
    // delay(100);
    // _max30001RegWrite(CNFG_CAL, 0x004800); // 0x700000
    // delay(100);
    // _max30001RegWrite(CNFG_EMUX, 0x000000);
    // delay(100);
    // _max30001RegWrite(CNFG_ECG, 0x805000); // d23 - d22 : 10 for 250sps , 00:500 sps
    // delay(100);
    // _max30001RegWrite(CNFG_RTOR1, 0x36A300);
    // delay(100);

    // _max30001RegWrite(CNFG_BMUX, 0x001040);
    // delay(100);
    // _max30001RegWrite(CNFG_BIOZ, 0x653070);
    // delay(100);

    // _max30001Synch();
    // delay(100);
}

// void MAX30001::BeginECGBioZ()
// {
//     max30001_cnfg_gen_t cnfg_gen;
//     max30001_cnfg_emux_t cnfg_emux;
//     max30001_cnfg_ecg_t cnfg_ecg;
//     max30001_cnfg_bmux_t cnfg_bmux;
//     max30001_cnfg_bioz_t cnfg_bioz;
    
//     // CONFIG GEN Register Settings
//     cnfg_gen.bit.en_ulp_lon = 0; // ULP Lead-ON Disabled
//     cnfg_gen.bit.fmstr = 0b00;
//     cnfg_gen.bit.en_ecg = 0b1;
//     cnfg_gen.bit.en_bioz = 0b1;
//     cnfg_gen.bit.en_bloff = 0x00;  // BioZ digital lead off detection disabled
//     cnfg_gen.bit.en_dcloff = 0x00; // DC Lead-Off Detection Disabled
//     cnfg_gen.bit.en_rbias = 0b00;  // RBias disabled
//     cnfg_gen.bit.rbiasv = 0b01;    // RBias =100 Mohm
//     cnfg_gen.bit.rbiasp = 0b00;    // RBias Positive Input not connected
//     cnfg_gen.bit.rbiasn = 0b00;    // RBias Negative Input not connected

//     // ECG Config Settings
//     cnfg_ecg.bit.rate = 0b10; // Default, 128SPS
//     cnfg_ecg.bit.gain = 0b10; // 160 V/V
//     cnfg_ecg.bit.dhpf = 0b1;  // 0.5Hz
//     cnfg_ecg.bit.dlpf = 0b01; // 40Hz
    
//     // ECG MUX Settings
//     cnfg_emux.bit.openp = 0;
//     cnfg_emux.bit.openn = 0;
//     cnfg_emux.bit.pol = 0;
//     cnfg_emux.bit.calp_sel = 0;
//     cnfg_emux.bit.caln_sel = 0;

//     // BioZ Config Settings
//     cnfg_bioz.bit.rate = 0;
//     cnfg_bioz.bit.ahpf = 0b010;
//     cnfg_bioz.bit.ext_rbias = 0x00;
//     cnfg_bioz.bit.ln_bioz = 1;
//     cnfg_bioz.bit.gain = 0b01; //(20 V/V)
//     cnfg_bioz.bit.dhpf = 0b010;
//     cnfg_bioz.bit.dlpf = 0x01;
//     cnfg_bioz.bit.fcgen = 0b0100;
//     cnfg_bioz.bit.cgmon = 0x00;
//     cnfg_bioz.bit.cgmag = 0b100;
//     cnfg_bioz.bit.phoff = 0x0011;

//     // BioZ MUX Settings
//     cnfg_bmux.bit.openp = 0;
//     cnfg_bmux.bit.openn = 0;
//     cnfg_bmux.bit.calp_sel = 0x00; // No cal signal on BioZ
//     cnfg_bmux.bit.caln_sel = 0x00; // No cal signal on BioZ
//     cnfg_bmux.bit.cg_mode = 0x00;  // Unchopped
//     cnfg_bmux.bit.en_bist = 0;
//     cnfg_bmux.bit.rnom = 0x00;
//     cnfg_bmux.bit.rmod = 0x04;
//     cnfg_bmux.bit.fbist = 0;

//     //_max30001RegWrite(MNGR_INT, 0x7B0000); // EFIT=16, BFIT=8
//     // delay(100);

//     // max30001SetInterrupts(EN_EINT | 0x01); // Enable ECG Interrupts

//     //_max30001RegWrite(CNFG_RTOR1,0x3fc600);

//     /* ****IMPORTANT: Write all registers in the same sequence**** */

//     _max30001SwReset();
//     delay(100);

//     _max30001RegWrite(CNFG_GEN, cnfg_gen.all);
//     //_max30001RegWrite(CNFG_GEN, 0xC0004); // ECG & BioZ Enabled , FMSTR = 32768
//     delay(100);

//     _max30001RegWrite(CNFG_CAL, 0x720000); // Calibration sources disabled
//     delay(100);

//     _max30001RegWrite(CNFG_ECG, cnfg_ecg.all);
//     delay(100);

//     _max30001RegWrite(CNFG_EMUX, cnfg_emux.all); // Pins internally connection to ECG Channels
//     delay(100);

//     _max30001RegWrite(CNFG_BIOZ, cnfg_bioz.all);
//     //_max30001RegWrite(CNFG_BIOZ, 0x201433); // BioZ Rate: 64 SPS | Current generator: 32 uA

//     // Set MAX30001G specific BioZ LC
//     _max30001RegWrite(CNFG_BIOZ_LC, 0x800000); // Turn OFF low current mode
//     delay(100);

//     _max30001RegWrite(CNFG_BMUX, 0x000040); // Pins connected internally to BioZ channels
//     delay(100);

//     _max30001Synch();
//     delay(100);
// }

void MAX30001::BeginRtoRMode()
{
    _max30001SwReset();
    delay(100);
    _max30001RegWrite(CNFG_GEN, 0x080004);
    delay(100);
    _max30001RegWrite(CNFG_CAL, 0x720000); // 0x700000
    delay(100);
    _max30001RegWrite(CNFG_EMUX, 0x0B0000);
    delay(100);
    _max30001RegWrite(CNFG_ECG, 0x805000); // d23 - d22 : 10 for 250sps , 00:500 sps
    delay(100);
    _max30001RegWrite(CNFG_RTOR1, 0x3fc600);
    delay(100);
    _max30001RegWrite(EN_INT, 0x000401);
    delay(100);
    _max30001Synch();
    delay(100);
}

// not tested
void MAX30001::max30001SetsamplingRate(uint16_t samplingRate)
{
    uint8_t regBuff[4] = {0};
    _max30001RegRead(CNFG_ECG, regBuff);

    switch (samplingRate)
    {
    case SAMPLINGRATE_128:
        regBuff[0] = (regBuff[0] | 0x80);
        break;

    case SAMPLINGRATE_256:
        regBuff[0] = (regBuff[0] | 0x40);
        break;

    case SAMPLINGRATE_512:
        regBuff[0] = (regBuff[0] | 0x00);
        break;

    default:
        Serial.println("Invalid sample rate. Please choose between 128, 256 or 512");
        break;
    }

    unsigned long cnfgEcg;
    memcpy(&cnfgEcg, regBuff, 4);

    Serial.print(" cnfg ECG ");
    Serial.println((cnfgEcg));
    _max30001RegWrite(CNFG_ECG, (cnfgEcg >> 8));
}

signed long MAX30001::getECGSamples(void)
{
    uint8_t regReadBuff[4];
    _max30001RegRead(ECG_FIFO, regReadBuff);

    /* FIFO format (zero padding needed to perform signed conversion): 
    *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    *  [                       DATA                        ] [ ETAG ] [ PTAG ] [       PADDING       ]
    */
    unsigned long byte0 = (unsigned long)(regReadBuff[0]);
    byte0 = byte0 << 24;
    unsigned long byte1 = (unsigned long)(regReadBuff[1]);
    byte1 = byte1 << 16;
    unsigned long byte2 = (unsigned long)(regReadBuff[2]);
    byte2 = byte2 << 8;
    unsigned long fifo = (unsigned long)(byte0 | byte1 | byte2);

    uint8_t e_tag = (fifo & 0x3800) >> 11;          // Extract ETAG[13:11], then right-shift 11 bits to align ETAG[2:0]. 0bXXX00000000000 -> 0b00000000000XXX
    if (e_tag == 0x07)
    {
      _max30001FIFOReset();
      // Serial.println("ECG FIFO overflow event. FIFO reset command sent. Internal write pointer caught up with read pointer. Recommend faster sampling.");
    } 
    else if (e_tag == 0x02)
    {
      // Serial.println("ECG FIFO End-of-File reached. Suspending read back operations on the ECG FIFO until more samples are available");
      delay(3);
    }

    ecg_data = (signed long)(fifo);
    return ecg_data;
}

signed long MAX30001::getBioZSamples(void)
{
    uint8_t regReadBuff[4];
    _max30001RegRead(BIOZ_FIFO, regReadBuff);

    /* FIFO format (zero padding needed to perform signed conversion): 
    *  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    *  [                          DATA                           ] [] [ BTAG ] [       PADDING       ]
    */
    unsigned long byte0 = (unsigned long)(regReadBuff[0]);
    byte0 = byte0 << 24;
    unsigned long byte1 = (unsigned long)(regReadBuff[1]);
    byte1 = byte1 << 16;
    unsigned long byte2 = (unsigned long)(regReadBuff[2]);
    byte2 = byte2 << 8;
    unsigned long fifo = (unsigned long)(byte0 | byte1 | byte2);

    uint8_t b_tag = (fifo >> 8) & 0x07;          // Right shift BTAG[10:08] to align 0bXXX00000000 -> 0b00000000XXX, then extract with 0b0111 mask.
    if (b_tag == 0x07)
    {
      _max30001FIFOReset();
      // Serial.println("BioZ FIFO overflow event. FIFO reset command sent. Internal write pointer caught up with read pointer. Recommend faster sampling.");
    } 
    else if (b_tag == 0x02)
    {
      // Serial.println("BioZ FIFO End-of-File reached. Suspending read back operations on the BioZ FIFO until more samples are available");
      delay(3);
    }
    
    bioz_data = (signed long)(fifo);
    return bioz_data;
}

void MAX30001::getHRandRR(void)
{
    uint8_t regReadBuff[4];
    _max30001RegRead(RTOR, regReadBuff);

    unsigned long RTOR_msb = (unsigned long)(regReadBuff[0]);
    unsigned char RTOR_lsb = (unsigned char)(regReadBuff[1]);
    unsigned long rtor = (RTOR_msb << 8 | RTOR_lsb);
    rtor = ((rtor >> 2) & 0x3fff);

    float hr = 60 / ((float)rtor * 0.0078125);
    heartRate = (unsigned int)hr;

    unsigned int RR = (unsigned int)rtor * (7.8125); // 8ms
    RRinterval = RR;
}

void MAX30001::max30001SetInterrupts(uint32_t interrupts_to_set)
{
    _max30001RegWrite(EN_INT, interrupts_to_set);
    delay(100);
    //_max30001Synch();
    // delay(100);
}

int secg_counter = 0;
int sbioz_counter = 0;

void MAX30001::_max30001ReadECGFIFO(int num_bytes)
{
    uint8_t spiTxBuff;
    unsigned long uecgtemp;
    signed long secgtemp;

    SPI.beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (ECG_FIFO_BURST << 1) | RREG;
    SPI.transfer(spiTxBuff); // Send register location

    for (int i = 0; i < num_bytes; i++)
    {
        _readBufferECG[i] = SPI.transfer(0x00);
    }

    digitalWrite(_cs_pin, HIGH);

    SPI.endTransaction();

    secg_counter = 0;
    unsigned char ecg_etag;

    for (int i = 0; i < num_bytes; i += 3)
    {
        // Get etag
        ecg_etag = ((((unsigned char)_readBufferECG[i + 2]) & 0x38) >> 3);
        // Serial.println(ecg_etag, HEX);

        if (ecg_etag == 0x00) // Valid sample
        {
            // uecgtemp=(unsigned long)((unsigned long)readBuffer[i]<<16 |(unsigned long)readBuffer[i+1]<<8| (unsigned long)(readBuffer[i+2]&0xC0));
            uecgtemp = (unsigned long)(((unsigned long)_readBufferECG[i] << 16 | (unsigned long)_readBufferECG[i + 1] << 8) | (unsigned long)(_readBufferECG[i + 2] & 0xC0));
            uecgtemp = (unsigned long)(uecgtemp << 8);

            secgtemp = (signed long)uecgtemp;
            secgtemp = (signed long)secgtemp >> 8;

            s32ECGData[secg_counter++] = secgtemp;
        }
        else if (ecg_etag == 0x07) // FIFO Overflow
        {
            // Serial.println("OVF");
            _max30001FIFOReset();
        }
    }

    // Serial.print("F");
    // Serial.println(secg_counter);

    ecgSamplesAvailable = secg_counter;
    secg_counter = 0;
}

void MAX30001::_max30001ReadBIOZFIFO(int num_bytes)
{
    uint8_t spiTxBuff;
    unsigned long ubioztemp;
    signed long sbioztemp;

    SPI.beginTransaction(SPISettings(MAX30001_SPI_SPEED, MSBFIRST, SPI_MODE0));

    digitalWrite(_cs_pin, LOW);

    spiTxBuff = (BIOZ_FIFO_BURST << 1) | RREG;
    SPI.transfer(spiTxBuff); // Send register location

    for (int i = 0; i < num_bytes; i++)
    {
        _readBufferBIOZ[i] = SPI.transfer(0x00);
    }

    digitalWrite(_cs_pin, HIGH);

    SPI.endTransaction();

    sbioz_counter = 0;
    unsigned char bioz_etag;

    for (int i = 0; i < num_bytes; i += 3)
    {
        // Get etag
        bioz_etag = ((((unsigned char)_readBufferBIOZ[i + 2]) & 0x38) >> 3);
        // Serial.println(ecg_etag, HEX);

        if (bioz_etag == 0x00) // Valid sample
        {
            // uecgtemp=(unsigned long)((unsigned long)readBuffer[i]<<16 |(unsigned long)readBuffer[i+1]<<8| (unsigned long)(readBuffer[i+2]&0xC0));
            ubioztemp = (unsigned long)(((unsigned long)_readBufferBIOZ[i] << 16 | (unsigned long)_readBufferBIOZ[i + 1] << 8) | (unsigned long)(_readBufferBIOZ[i + 2] & 0xC0));
            ubioztemp = (unsigned long)(ubioztemp << 8);

            sbioztemp = (signed long)ubioztemp;
            sbioztemp = (signed long)sbioztemp >> 8;

            s32BIOZData[sbioz_counter++] = sbioztemp;
        }
        else if (bioz_etag == 0x07) // FIFO Overflow
        {
            // Serial.println("OVF");
            _max30001FIFOReset();
        }
    }

    biozSamplesAvailable = sbioz_counter;
    sbioz_counter = 0;
}

void MAX30001::max30001ServiceAllInterrupts(void)
{
    static uint32_t InitReset = 0;
    int fifo_num_bytes = 0;

    max30001_mngr_int_t mngr_int;

    _max30001RegRead24(STATUS, &global_status.all);

    if (global_status.bit.eint == 1) // EINT bit is set. FIFO is full
    {
        // Read the number of bytes in FIFO (from  MNGR_INT register)
        _max30001RegRead24(MNGR_INT, &mngr_int.all);
        fifo_num_bytes = (mngr_int.bit.e_fit + 1) * 3;

        _max30001ReadECGFIFO(fifo_num_bytes);
    }

    if (global_status.bit.bint == 1) // BIOZ FIFO is full
    {
        _max30001RegRead24(MNGR_INT, &mngr_int.all);
        fifo_num_bytes = (mngr_int.bit.b_fit + 1) * 3;

        // Read BIOZ FIFO in Burst mode
        _max30001ReadBIOZFIFO(fifo_num_bytes);
    }
}