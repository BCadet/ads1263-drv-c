#include "ads1263.h"

/*!
\brief Reset ADS126X by control RESET pin
\param [out] ads1263 Initialized variable of type ads1263_t
*/
void ADS1263_HardReset(ads1263_t *ads1263)
{
    ads1263->SetReset(0);
    ads1263->DelayMs(1000);
    ads1263->SetReset(1);
}

/*!
\brief Reset ADS126X by command sequence
\param [out] ads1263 Initialized variable of type ads1263_t
*/
void ADS1263_SoftReset(ads1263_t *ads1263)
{
    // Soft reset not implemented. Use hard reset instead
    ADS1263_HardReset(ads1263);
}

/*!
\brief Init ADS126X according to datasheet's sequence
\param [out] ads1263 Initialized variable of type ads1263_t
*/
void ADS1263_Init(ads1263_t *ads1263)
{
    if (ads1263->DelayMs == 0 || ads1263->SetReset == 0)
    {
        return;
    }
    // Hard reset
    ADS1263_HardReset(ads1263);
    // ADS1263_SoftReset(ads1263);
    ads1263->DelayMs(100);

    // Reset indicator setup
    ADS1263_SetPowerState(ads1263, ADS1263_POWER_SETUP);
}

/*!
\brief Function for reading register data
\param [in] regAddress Address of register to read
\return Value of wanted register
*/

uint8_t ADS1263_ReadReg(ads1263_t *ads1263, uint8_t regAddress)
{
    uint8_t data = 0;
    uint8_t readCmd[4] = {0};
    uint8_t rx[4] = {0};

    readCmd[0] = ADS1263_READ_ADD | regAddress;
    readCmd[1] = 0x00; // according to datasheet (OPCODE2 byte for RREG Command for one register)
    readCmd[2] = 0x00;
    readCmd[3] = 0x00;

    ads1263->SetCS(0);
    ads1263->Transfer(readCmd, rx, 4);
    ads1263->SetCS(1);

    data = rx[2];

    return data;
}

/*!
\brief Function for writing data to register
\param [in] regAddress Address of register to read
\param [in] data[] Data
*/

void ADS1263_WriteReg(ads1263_t *ads1263, uint8_t regAddress, uint8_t data)
{
    uint8_t writeCmd[3] = {0};
    uint8_t rx[3] = {0};

    writeCmd[0] = ADS1263_WRITE_ADD | regAddress;
    writeCmd[1] = 0x00; // according to datasheet (OPCODE2 byte for WREG Command for one register)
    writeCmd[2] = data;

    ads1263->SetCS(0);
    ads1263->Transfer(writeCmd, rx, 3);
    ads1263->SetCS(1);
}

/*!
\brief Function for reading the ADC value from the register.
*/

uint32_t ADS1263_ReadAdc(ads1263_t *ads1263, uint8_t adc)
{
    uint32_t msg = 0;
    uint8_t readCmd[7] = {0};
    uint8_t rx[7] = {0};

    readCmd[0] = (adc == 2) ? ADS1263_RDATA2_CMD : ADS1263_RDATA1_CMD;
    readCmd[1] = 0x00; // according to datasheet (OPCODE2 byte for RREG Command for one register)
    readCmd[2] = 0x00;
    readCmd[3] = 0x00;
    readCmd[4] = 0x00;
    readCmd[5] = 0x00;
    readCmd[6] = 0x00;

    ads1263->SetCS(0);
    ads1263->Transfer(readCmd, rx, 7);
    ads1263->SetCS(1);

    ads1263->status.adc2 = (rx[1] & 0x80) >> 7;
    ads1263->status.adc1 = (rx[1] & 0x40) >> 6;
    ads1263->status.extclk = (rx[1] & 0x20) >> 5;
    ads1263->status.ref_alm = (rx[1] & 0x10) >> 4;
    ads1263->status.pgal_alm = (rx[1] & 0x08) >> 3;
    ads1263->status.pgah_alm = (rx[1] & 0x04) >> 2;
    ads1263->status.pgad_alm = (rx[1] & 0x02) >> 1;
    ads1263->status.reset = (rx[1] & 0x01);

    msg |= (rx[2] << 24) & 0xFF000000;
    msg |= (rx[3] << 16) & 0xFF0000;
    msg |= (rx[4] << 8) & 0xFF00;
    msg |= (rx[5]) & 0xFF;

    return msg;
}

void ADS1263_SendCommand(ads1263_t *ads1263, uint8_t command)
{
    uint8_t writeCmd[3] = {0};
    uint8_t rx[3] = {0};

    writeCmd[0] = command;
    writeCmd[1] = 0x00; // according to datasheet (OPCODE2 byte for WREG Command for one register)
    writeCmd[2] = 0x00;

    ads1263->SetCS(0);
    ads1263->Transfer(writeCmd, rx, 3);
    ads1263->SetCS(1);
}

/*!
\brief Function for triggering ADC1 conversion.
*/

void ADS1263_StartAdc1(ads1263_t *ads1263)
{
    if (ads1263->SetStart != 0)
    {
        ads1263->SetStart(1);
    }
    else
    {
        ADS1263_SendCommand(ads1263, ADS1263_START1_CMD);
    }
}

/*!
\brief Function for stopping ADC1 conversion.
*/

void ADS1263_StopAdc1(ads1263_t *ads1263)
{
    if (ads1263->SetStart != 0)
    {
        ads1263->SetStart(0);
        return;
    }
    ADS1263_SendCommand(ads1263, ADS1263_STOP1_CMD);
}

/******************************************************************************/

/*!
\brief Function for triggering ADC2 conversion.
*/

void ADS1263_StartAdc2(ads1263_t *ads1263)
{
    ADS1263_SendCommand(ads1263, ADS1263_START2_CMD);
}

/*!
\brief Function for stopping ADC2 conversion.
*/

void ADS1263_StopAdc2(ads1263_t *ads1263)
{
    ADS1263_SendCommand(ads1263, ADS1263_STOP2_CMD);
}

/******************************************************************************/

/*!
\brief Function for checking Reset state. If device reset occurred - the reinitialization is performed
*/

void ADS1263_CheckReset(ads1263_t *ads1263)
{
    ADS1263_GetPowerState(ads1263);
    if (ads1263->power.reset == 0x01)
    {
        ADS1263_Init(ads1263);
    }
}

/* -------- Setting Register Value Functions Section -------- */

/*!
\brief Function for setting  IDAC Multiplexer Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetIDACMuxState(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_IDACMUX, regVal);
}

/*!
\brief Function for setting  IDAC Magnitude Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetIDACMagState(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_IDACMAG, regVal);
}

/*!
\brief Function for setting Input Multiplexer Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetInputMuxState(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_INPMUX, regVal);
}

/*!
\brief Function for setting Power Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetPowerState(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_POWER, regVal);
}

/*!
\brief Function for setting Mode0 Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetMode0State(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_MODE0, regVal);
}

/*!
\brief Function for setting Mode1 Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetMode1State(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_MODE1, regVal);
}

/*!
\brief Function for setting Mode2 Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetMode2State(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_MODE2, regVal);
}

/*!
\brief Function for setting TDAC Positive Output Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetTDACPState(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_TDACP, regVal);
}

/*!
\brief Function for setting TDAC Negative Output Register value
\param [in] regVal Value of register to set
*/

void ADS1263_SetTDACNState(ads1263_t *ads1263, uint8_t regVal)
{
    ADS1263_WriteReg(ads1263, ADS1263_TDACN, regVal);
}

/* ---------------------------------------------------------- */

/* -------- Reading Register Data Functions Section -------- */

/*!
\brief Function for getting Device Identification Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetIdState(ads1263_t *ads1263)
{
    ads1263->id.reg = ADS1263_ReadReg(ads1263, ADS1263_ID);
}

/*!
\brief Function for getting Power Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetPowerState(ads1263_t *ads1263)
{
    ads1263->power.reg = ADS1263_ReadReg(ads1263, ADS1263_POWER);
}

/*!
\brief Function for getting Interface Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetInterfaceState(ads1263_t *ads1263)
{
    ads1263->interface.reg = ADS1263_ReadReg(ads1263, ADS1263_INTERFACE);
}

/*!
\brief Function for getting Mode0 Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetMode0State(ads1263_t *ads1263)
{
    ads1263->mode0.reg = ADS1263_ReadReg(ads1263, ADS1263_MODE0);
}

/*!
\brief Function for getting Mode1 Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetMode1State(ads1263_t *ads1263)
{
    ads1263->mode1.reg = ADS1263_ReadReg(ads1263, ADS1263_MODE1);
}

/*!
\brief Function for getting Mode2 Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetMode2State(ads1263_t *ads1263)
{
    ads1263->mode2.reg = ADS1263_ReadReg(ads1263, ADS1263_MODE2);
}

/*!
\brief Function for getting Input Multiplexer Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetInputMuxState(ads1263_t *ads1263)
{
    ads1263->inpmux.reg = ADS1263_ReadReg(ads1263, ADS1263_INPMUX);
}

/*!
\brief Function for getting Offset Calibration Register data

Three registers compose the 24-bit offset calibration word. The
24-bit word is twos complement format, and is internally left-
shifted to align with the 32-bit conversion result. The ADC
subtracts the register value from the 32-bit conversion result
before the full-scale operation.

\param [out] ads1263 Initialized variable of type ads1263_t
\warning final word for register: LSB - OFCAL0 register data, MSB - OFCAL2 register data. Need to test
*/

void ADS1263_GetOffsetCalState(ads1263_t *ads1263)
{
    ads1263->ofcal.ofcal0 = ADS1263_ReadReg(ads1263, ADS1263_OFCAL0);
    ads1263->ofcal.ofcal1 = ADS1263_ReadReg(ads1263, ADS1263_OFCAL1);
    ads1263->ofcal.ofcal2 = ADS1263_ReadReg(ads1263, ADS1263_OFCAL2);
}

/*!
\brief Function for getting Full-Scale Calibration Register data

Three 8-bit registers compose the 24-bit full scale calibration
word. The 24-bit word format is straight binary. The ADC divides
the 24-bit value by 400000h to derive the gain coefficient. The
ADC multiplies the gain coefficient by the 32-bit conversion
result after the offset operation.

\param [out] ads1263 Initialized variable of type ads1263_t
\warning final word for register: LSB - FSCAL0 register data, MSB - FSCAL2 register data. Need to test
*/

void ADS1263_GetFSCalState(ads1263_t *ads1263)
{
    ads1263->fscal.fscal0 = ADS1263_ReadReg(ads1263, ADS1263_FSCAL0);
    ads1263->fscal.fscal1 = ADS1263_ReadReg(ads1263, ADS1263_FSCAL1);
    ads1263->fscal.fscal2 = ADS1263_ReadReg(ads1263, ADS1263_FSCAL2);
}

/*!
\brief Function for getting  IDAC Multiplexer Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetIDACMuxState(ads1263_t *ads1263)
{
    ads1263->idacmux.reg = ADS1263_ReadReg(ads1263, ADS1263_IDACMUX);
}

/*!
\brief Function for getting  IDAC Magnitude Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetIDACMagState(ads1263_t *ads1263)
{
    ads1263->idacmag.reg = ADS1263_ReadReg(ads1263, ADS1263_IDACMAG);
}

/*!
\brief Function for getting Reference Multiplexer Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetRefMuxState(ads1263_t *ads1263)
{
    ads1263->refmux.reg = ADS1263_ReadReg(ads1263, ADS1263_REFMUX);
}

/*!
\brief Function for getting TDAC Positive Output Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetTDACPState(ads1263_t *ads1263)
{
    ads1263->tdacp.reg = ADS1263_ReadReg(ads1263, ADS1263_TDACP);
}

/*!
\brief Function for getting TDAC Negative Output Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetTDACNState(ads1263_t *ads1263)
{
    ads1263->tdacn.reg = ADS1263_ReadReg(ads1263, ADS1263_TDACN);
}

/*!
\brief Function for getting  GPIO Connection Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetGpioConState(ads1263_t *ads1263)
{
    ads1263->gpiocon.reg = ADS1263_ReadReg(ads1263, ADS1263_GPIOCON);
}

/*!
\brief Function for getting  GPIO Direction Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetGpioDirState(ads1263_t *ads1263)
{
    ads1263->gpiodir.reg = ADS1263_ReadReg(ads1263, ADS1263_GPIODIR);
}

/*!
\brief Function for getting  GPIO Data Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetGpioDatState(ads1263_t *ads1263)
{
    ads1263->gpiodat.reg = ADS1263_ReadReg(ads1263, ADS1263_GPIODAT);
}

/*!
\brief Function for getting ADC2 Configuration Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetAdc2CfgState(ads1263_t *ads1263)
{
    ads1263->adc2cfg.reg = ADS1263_ReadReg(ads1263, ADS1263_ADC2CFG);
}

/*!
\brief Function for getting ADC2 Input Multiplexer Register data
\param [out] ads1263 Initialized variable of type ads1263_t
*/

void ADS1263_GetAdc2MuxState(ads1263_t *ads1263)
{
    ads1263->adc2mux.reg = ADS1263_ReadReg(ads1263, ADS1263_ADC2MUX);
}

/*!
\brief Function for getting  ADC2 Offset Calibration Registers data.

Two registers compose the ADC2 16-bit offset calibration word.
The 16-bit word is twos complement format and is internally left-shifted to align with the ADC2 24-bit conversion result.
The ADC subtracts the register value from the conversion result before full-scale operation.

\param [out] ads1263 Initialized variable of type ads1263_t
\warning final word for register: LSB - ADC2OFC0 register data, MSB - ADC2OFC1 register data. Need to test
*/

void ADS1263_GetAdc2OffsetCalState(ads1263_t *ads1263)
{
    ads1263->adc2ofc.adc2ofc0 = ADS1263_ReadReg(ads1263, ADS1263_ADC2OFC0);
    ads1263->adc2ofc.adc2ofc1 = ADS1263_ReadReg(ads1263, ADS1263_ADC2OFC1);
}

/*!
\brief Function for getting  ADC2 Offset Calibration Registers data.

Two registers compose the ADC2 16-bit full scale calibration word. The 16-bit word format is straight binary.
The ADC divides the 16-bit value by 4000h to derive the scale factor for calibration.
After the offset operation, the ADC multiplies the scale factor by the conversion result.

\param [out] ads1263 Initialized variable of type ads1263_t
\warning final word for register: LSB - ADC2FSC0 register data, MSB - ADC2FSC1 register data. Need to test

*/

void ADS1263_GetAdc2FSCalState(ads1263_t *ads1263)
{
    ads1263->adc2fsc.adc2fsc0 = ADS1263_ReadReg(ads1263, ADS1263_ADC2FSC0);
    ads1263->adc2fsc.adc2fsc1 = ADS1263_ReadReg(ads1263, ADS1263_ADC2FSC1);
}

/* --------------------------------------------------------- */

void ADS1263_DumpRegisters(ads1263_t *ads1263)
{
    ADS1263_GetIdState(ads1263);
    ADS1263_GetPowerState(ads1263);
    ADS1263_GetInterfaceState(ads1263);
    ADS1263_GetMode0State(ads1263);
    ADS1263_GetMode1State(ads1263);
    ADS1263_GetMode2State(ads1263);
    ADS1263_GetInputMuxState(ads1263);
    ADS1263_GetOffsetCalState(ads1263);
    ADS1263_GetFSCalState(ads1263);
    ADS1263_GetIDACMuxState(ads1263);
    ADS1263_GetIDACMagState(ads1263);
    ADS1263_GetRefMuxState(ads1263);
    ADS1263_GetTDACPState(ads1263);
    ADS1263_GetTDACNState(ads1263);
    ADS1263_GetGpioConState(ads1263);
    ADS1263_GetGpioDirState(ads1263);
    ADS1263_GetGpioDatState(ads1263);
    ADS1263_GetAdc2CfgState(ads1263);
    ADS1263_GetAdc2MuxState(ads1263);
    ADS1263_GetAdc2OffsetCalState(ads1263);
    ADS1263_GetAdc2FSCalState(ads1263);

    printf("\r\nADS1263 registers");
    printf("\r\nid\t%02x", ads1263->id.reg);
    printf("\r\npower\t%02x", ads1263->power.reg);
    printf("\r\ninterface\t%02x", ads1263->interface.reg);
    printf("\r\nmode0\t%02x", ads1263->mode0.reg);
    printf("\r\nmode1\t%02x", ads1263->mode1.reg);
    printf("\r\nmode2\t%02x", ads1263->mode2.reg);
    printf("\r\ninpmux\t%02x", ads1263->inpmux.reg);
    printf("\r\nofcal\t%06x", ads1263->ofcal);
    printf("\r\nfscal\t%06x", ads1263->fscal);
    printf("\r\nidacmux\t%02x", ads1263->idacmux.reg);
    printf("\r\nidacmag\t%02x", ads1263->idacmag.reg);
    printf("\r\nrefmux\t%02x", ads1263->refmux.reg);
    printf("\r\ntdacp\t%02x", ads1263->tdacp.reg);
    printf("\r\ntdacn\t%02x", ads1263->tdacn.reg);
    printf("\r\ngpiocon\t%02x", ads1263->gpiocon.reg);
    printf("\r\ngpiodir\t%02x", ads1263->gpiodir.reg);
    printf("\r\ngpiodat\t%02x", ads1263->gpiodat.reg);
    printf("\r\nadc2cfg\t%02x", ads1263->adc2cfg.reg);
    printf("\r\nadc2mux\t%02x", ads1263->adc2mux.reg);
    printf("\r\nadc2ofc\t%04x", ads1263->adc2ofc);
    printf("\r\nadc2fsc\t%04x\r\n", ads1263->adc2fsc.fsc2);
}