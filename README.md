# ads1263-drv-c

\anchor ads1263-drv-c

Object oriented C driver implemenation for the [ADS1263](http://www.ti.com/product/ADS1263)

## Licence

BSD-2 Clause. See LICENCE file for additional information

## Description

* ***./ads1263.h*** - header file of the driver 
* ***./ads1263.c*** - code file of the driver 
* ***./LICENCE*** - licence file
* ***./CMakeLists.txt*** - CMake file
* ***./README.md*** - this file  

## Embedding information

For work with this driver you should follow these steps:

1. Include this repo to your project (manually, with CMake 'add_subdirectory', with CMake 'FetchContent')

2. link the library `ads1263` to your application if you use CMake

3. Define and implement following functions with signatures (names of functions may be specific for you):

    * ```spiTransfer(uint8_t tx, uint8_t rx, uint8_t len)``` - transfer data over SPI

    * ```setCS(uint8_t state)``` - control CS line. When ```state``` equals to ```1``` a state of line is *HIGH*

    * ```setReset(uint8_t state)``` - control RESET line. When ```state``` equals to ```1``` a state of line is *HIGH*

    * ```setStart(uint8_t state)``` - control START line. When ```state``` equals to ```1``` a state of line is *HIGH*

    * ```DelayMs(uint32_t delay)``` - delay in milliseconds

4. Init main structure with pointers to your implemented functions at prevous step:

    ```c
    //object declaration
    ads1263_t ads1263;

    //populate object pointers
    ads1263.Transfer = spiTransfer;
    ads1263.SetCS = setCS;
    ads1263.SetReset = setReset;
    ads1263.SetStart = setStart;
    ads1263.DelayMs = DelayMs;

    // ads126 initialization
    ADS1263_Init(&ads1263);
    ```
