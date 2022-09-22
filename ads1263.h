#ifndef INC_ADS1263_H_
#define INC_ADS1263_H_

#include <stdint.h>

enum ADS1263_AINMUX {
    ADS1263_AINMUX_AIN0 = 0,
    ADS1263_AINMUX_AIN1 = 1,
    ADS1263_AINMUX_AIN2 = 2,
    ADS1263_AINMUX_AIN3 = 3,
    ADS1263_AINMUX_AIN4 = 4,
    ADS1263_AINMUX_AIN5 = 5,
    ADS1263_AINMUX_AIN6 = 6,
    ADS1263_AINMUX_AIN7 = 7,
    ADS1263_AINMUX_AIN8 = 8,
    ADS1263_AINMUX_AIN9 = 9,
    ADS1263_AINMUX_AINCOM = 10,
    ADS1263_AINMUX_TEMP = 11,
    ADS1263_AINMUX_ANALOG_POWER = 12,
    ADS1263_AINMUX_DIGITAL_POWER = 13,
    ADS1263_AINMUX_TDAC = 14,
    ADS1263_AINMUX_FLOAT = 15,
};

#define MUXP(x) ((x<<4) & 0xF0)
#define MUXN(x) ((x) & 0x0F)

#define MUX(ainmuxP, ainmuxN) (MUXP(ainmuxP) | MUXN(ainmuxN))

/* ____________________ DEFINE Section ____________________ */
/* Register addresses */
#define ADS1263_ID              (0x00)
#define ADS1263_POWER           (0x01)
#define ADS1263_INTERFACE       (0x02)
#define ADS1263_MODE0           (0x03)
#define ADS1263_MODE1           (0x04)
#define ADS1263_MODE2           (0x05)
#define ADS1263_INPMUX          (0x06)
#define ADS1263_OFCAL0          (0x07)
#define ADS1263_OFCAL1          (0x08)
#define ADS1263_OFCAL2          (0x09)
#define ADS1263_FSCAL0          (0x0A)
#define ADS1263_FSCAL1          (0x0B)
#define ADS1263_FSCAL2          (0x0C)
#define ADS1263_IDACMUX         (0x0D)
#define ADS1263_IDACMAG         (0x0E)
#define ADS1263_REFMUX          (0x0F)
#define ADS1263_TDACP           (0x10)
#define ADS1263_TDACN           (0x11)
#define ADS1263_GPIOCON         (0x12)
#define ADS1263_GPIODIR         (0x13)
#define ADS1263_GPIODAT         (0x14)
#define ADS1263_ADC2CFG         (0x15)
#define ADS1263_ADC2MUX         (0x16)
#define ADS1263_ADC2OFC0        (0x17)
#define ADS1263_ADC2OFC1        (0x18)
#define ADS1263_ADC2FSC0        (0x19)
#define ADS1263_ADC2FSC1        (0x1A)

/* Register settings */
#define ADS1263_POWER_SETUP     (0x01)      //Set Reset indicator to 0
#define ADS1263_POWER_DEFAULT   (0x11)      //Set Reset indicator to 1 (Default state)
#define ADS1263_INPMUX_DEFAULT  (0x01)      //Default MUX setup (MUXP - AIN0 and MUXN - AIN1)
#define ADS1263_INPMUX_SETUP    (0x23)      //MUXP - AIN2 and MUXN - AIN3
#define ADS1263_IDACMUX_SETUP   (0xB4)      //IDAC1 and AIN4
#define ADS1263_IDACMUX_DEFAULT (0xBB)      //IDAC disabled (Default state)
#define ADS1263_IDACMAG_SETUP   (0x06)      //IDAC1 and 1 mA
#define ADS1263_IDACMAG_DEFAULT (0x00)      //IDAC disabled (Default state)
#define ADS1263_MODE0_SETUP     (0x40)      //Pulse conversion (one shot)
#define ADS1263_MODE0_DEFAULT   (0x00)      //Continuous conversion (default)
#define ADS1263_MODE1_SINC1     (0x00)      //Sinc1 mode
#define ADS1263_MODE1_SINC2     (0x20)      //Sinc2 mode
#define ADS1263_MODE1_SINC3     (0x40)      //Sinc3 mode
#define ADS1263_MODE1_SINC4     (0x60)      //Sinc4 mode
#define ADS1263_MODE1_DEFAULT   (0x80)      //FIR mode (default)
#define ADS1263_MODE2_SETUP     (0x09)      //PGA bypass enabled, 1V/V, 1200 SPS (Not working with FIR)
#define ADS1263_MODE2_DEFAULT   (0x04)      //PGA bypass enabled, 1V/V, 20 SPS (default)
#define ADS1263_TDACP_SETUP     (0x80)      //Set TDACP output to pin AIN6 (2.5 V)
#define ADS1263_TDACP_DEFAULT   (0x00)      //Default state (off)
#define ADS1263_TDACN_SETUP     (0x98)      //Set TDACP output to pin AIN7 (1.5 V)
#define ADS1263_TDACN_DEFAULT   (0x00)      //Default state (off)

/* Commands */
#define ADS1263_NOP_CMD         (0x00)
#define ADS1263_RESET_CMD       (0x06)      //Or 0x07
#define ADS1263_START1_CMD      (0x08)      //Or 0x09
#define ADS1263_STOP1_CMD       (0x0A)      //Or 0x0B
#define ADS1263_START2_CMD      (0x0C)      //Or 0x0D
#define ADS1263_STOP2_CMD       (0x0E)      //Or 0x0F
#define ADS1263_RDATA1_CMD      (0x12)      //Or 0x13
#define ADS1263_RDATA2_CMD      (0x14)      //Or 0x15
#define ADS1263_SYOCAL1_CMD     (0x16)
#define ADS1263_SYGCAL1_CMD     (0x17)
#define ADS1263_SFOCAL1_CMD     (0x19)
#define ADS1263_SYOCAL2_CMD     (0x1B)
#define ADS1263_SYGCAL2_CMD     (0x1C)
#define ADS1263_SFOCAL2_CMD     (0x1E)


/* Const */
#define ADS1263_READ_ADD        (0x20)
#define ADS1263_WRITE_ADD       (0x40)

#define ADS1263_HIGH            (1)
#define ADS1263_LOW             (0)
/* ________________________________________________________ */

/* ____________________ Types  Section ____________________ */

typedef union {
    struct __attribute__((packed)) {
    uint8_t revId:5;
	uint8_t devId:3;
    };
    uint8_t reg;
} ads1263_id_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t intRef:1;
    uint8_t vBias:1;
    uint8_t __reserved:2;
	uint8_t reset:1;
    uint8_t _reserved:3;
    };
    uint8_t reg;
} ads1263_power_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t crc:2;
    uint8_t status:1;
    uint8_t timeOut:1;
    uint8_t _reserved:4;
    };
    uint8_t reg;
} ads1263_interface_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t delay:4;
    uint8_t chop:2;
    uint8_t runMode:1;
    uint8_t refRev:1;
    };
    uint8_t reg;
} ads1263_mode0_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t sBMag:3;
    uint8_t sBPol:1;
    uint8_t sBADC:1;
    uint8_t filter:3;
    };
    uint8_t reg;
} ads1263_mode1_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t dr:4;
    uint8_t gain:3;
    uint8_t byPass:1;
    };
    uint8_t reg;
} ads1263_mode2_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t muxN:4;
    uint8_t muxP:4;
    };
    uint8_t reg;
} ads1263_inpmux_t;

typedef union {
    struct __attribute__((packed)) {
        uint8_t ofcal0;
        uint8_t ofcal1;
        uint8_t ofcal2;
        uint8_t _reserved;
    };
    uint32_t ofc;
} ads1263_ofcal_t;

typedef union {
struct __attribute__((packed)) {
    uint8_t fscal0;
    uint8_t fscal1;
    uint8_t fscal2;
    uint8_t _reserved;
};
    uint32_t fscal;
} ads1263_fscal_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t mux2:4;
    uint8_t mux1:4;
    };
    uint8_t reg;
} ads1263_idacmux_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t mag2:4;
    uint8_t mag1:4;
    };
    uint8_t reg;
} ads1263_idacmag_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t rMuxN:3;
    uint8_t rMuxP:3;
    uint8_t _reserved:2;
    };
    uint8_t reg;
} ads1263_refmux_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t magP:5;
    uint8_t _reserved:2;
    uint8_t outP:1;
    };
    uint8_t reg;
} ads1263_tdacp_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t magN:5;
    uint8_t _reserved:2;
    uint8_t outN:1;
    };
    uint8_t reg;
} ads1263_tdacn_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t con7:1;
    uint8_t con6:1;
    uint8_t con5:1;
    uint8_t con4:1;
    uint8_t con3:1;
    uint8_t con2:1;
    uint8_t con1:1;
    uint8_t con0:1;
    };
    uint8_t reg;
} ads1263_gpiocon_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t dir7:1;
    uint8_t dir6:1;
    uint8_t dir5:1;
    uint8_t dir4:1;
    uint8_t dir3:1;
    uint8_t dir2:1;
    uint8_t dir1:1;
    uint8_t dir0:1;
    };
    uint8_t reg;
} ads1263_gpiodir_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t dat7:1;
    uint8_t dat6:1;
    uint8_t dat5:1;
    uint8_t dat4:1;
    uint8_t dat3:1;
    uint8_t dat2:1;
    uint8_t dat1:1;
    uint8_t dat0:1;
    };
    uint8_t reg;
} ads1263_gpiodat_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t gain2:3;
    uint8_t ref2:3;
    uint8_t dr2:2;
    };
    uint8_t reg;
} ads1263_adc2cfg_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t muxN2:4;
    uint8_t muxP2:4;
    };
    uint8_t reg;
} ads1263_adc2mux_t;

typedef union {
    struct __attribute__((packed)) {
        uint8_t adc2ofc0;
        uint8_t adc2ofc1;
    };
    uint16_t ofc2;
} ads1263_adc2ofc_t;

typedef union {
    struct __attribute__((packed)) {
        uint8_t adc2fsc0;
        uint8_t adc2fsc1;
    };
    uint16_t fsc2;
} ads1263_adc2fsc_t;

typedef union {
    struct __attribute__((packed)) {
    uint8_t reset:1;
    uint8_t pgad_alm:1;
    uint8_t pgah_alm:1;
    uint8_t pgal_alm:1;
    uint8_t ref_alm:1;
    uint8_t extclk:1;
    uint8_t adc1:1;
    uint8_t adc2:1;
    };
    uint8_t reg;
} ads1263_status_t;



typedef struct {
    void (*DelayMs)(uint32_t delay);
    void (*Transfer)(uint8_t tx[], uint8_t rx[], uint8_t len);
    void (*SetCS)(uint8_t state);
    void (*SetReset)(uint8_t state);
    void (*SetStart)(uint8_t state);

    ads1263_id_t id;
    ads1263_power_t power;
    ads1263_interface_t interface;
    ads1263_mode0_t mode0;
    ads1263_mode1_t mode1;
    ads1263_mode2_t mode2;
    ads1263_inpmux_t inpmux;
    ads1263_ofcal_t ofcal;
    ads1263_fscal_t fscal;
    ads1263_idacmux_t idacmux;
    ads1263_idacmag_t idacmag;
    ads1263_refmux_t refmux;
    ads1263_tdacp_t tdacp;
    ads1263_tdacn_t tdacn;
    ads1263_gpiocon_t gpiocon;
    ads1263_gpiodir_t gpiodir;
    ads1263_gpiodat_t gpiodat;
    ads1263_adc2cfg_t adc2cfg;
    ads1263_adc2mux_t adc2mux;
    ads1263_adc2ofc_t adc2ofc;
    ads1263_adc2fsc_t adc2fsc;
    ads1263_status_t status;
} ads1263_t;

/* ________________________________________________________ */





/* __________________ Prototypes Section __________________ */
void ADS1263_HardReset(ads1263_t * ads1263);
void ADS1263_SoftReset(ads1263_t * ads1263);
void ADS1263_Init(ads1263_t * ads1263);

void ADS1263_CheckReset(ads1263_t * ads1263);
void ADS1263_StartAdc1(ads1263_t * ads1263);
void ADS1263_StopAdc1(ads1263_t * ads1263);
void ADS1263_StartAdc2(ads1263_t * ads1263);
void ADS1263_StopAdc2(ads1263_t * ads1263);
uint32_t ADS1263_ReadAdc(ads1263_t * ads1263, uint8_t adc);
uint8_t ADS1263_ReadReg(ads1263_t * ads1263, uint8_t regAddress);
void ADS1263_WriteReg(ads1263_t * ads1263, uint8_t regAddress, uint8_t data);

/*          Reading Register Data Functions Section          */
void ADS1263_GetIdState(ads1263_t * ads1263);
void ADS1263_GetPowerState(ads1263_t * ads1263);
void ADS1263_GetInterfaceState(ads1263_t * ads1263);
void ADS1263_GetMode0State(ads1263_t * ads1263);
void ADS1263_GetMode1State(ads1263_t * ads1263);
void ADS1263_GetMode2State(ads1263_t * ads1263);
void ADS1263_GetInputMuxState(ads1263_t * ads1263);
void ADS1263_GetOffsetCalState(ads1263_t * ads1263);
void ADS1263_GetFSCalState(ads1263_t * ads1263);
void ADS1263_GetIDACMuxState(ads1263_t * ads1263);
void ADS1263_GetIDACMagState(ads1263_t * ads1263);
void ADS1263_GetRefMuxState(ads1263_t * ads1263);
void ADS1263_GetTDACPState(ads1263_t * ads1263);
void ADS1263_GetTDACNState(ads1263_t * ads1263);
void ADS1263_GetGpioConState(ads1263_t * ads1263);
void ADS1263_GetGpioDirState(ads1263_t * ads1263);
void ADS1263_GetGpioDatState(ads1263_t * ads1263);
void ADS1263_GetAdc2CfgState(ads1263_t * ads1263);
void ADS1263_GetAdc2MuxState(ads1263_t * ads1263);
void ADS1263_GetAdc2OffsetCalState(ads1263_t * ads1263);
void ADS1263_GetAdc2FSCalState(ads1263_t * ads1263);
/* --------------------------------------------------------- */

/*          Setting Register Data Functions Section          */
void ADS1263_SetPowerState(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetInputMuxState(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetIDACMuxState(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetIDACMagState(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetMode0State(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetMode1State(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetMode2State(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetTDACPState(ads1263_t * ads1263, uint8_t regVal);
void ADS1263_SetTDACNState(ads1263_t * ads1263, uint8_t regVal);
/* --------------------------------------------------------- */

void ADS1263_DumpRegisters(ads1263_t * ads1263);

/* ________________________________________________________ */


#endif /* INC_ADS1263_H_ */
