
#ifndef __ADS1298_H
#define __ADS1298_H

#include "stm32f4xx_hal.h"


#define ADS_OPCODE_WAKEUP 	0x02 // Wake-up from standby mode
#define ADS_OPCODE_STANDBY 	0x04 // Enter Standby mode
#define ADS_OPCODE_RESET 	0x06 // Reset the device registers to default
#define ADS_OPCODE_START    0x08 // Start and restart (synchronize) conversions
#define ADS_OPCODE_STOP   	0x0A // Stop conversion
#define ADS_OPCODE_RDATAC 	0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define ADS_OPCODE_SDATAC 	0x11 // Stop Read Data Continuous mode
#define ADS_OPCODE_RDATA    0x12 // Read data by command; supports multiple read back

//Register Addresses
#define ADS_ADR_ID          0x00
#define ADS_ADR_CONFIG1     0x01
#define ADS_ADR_CONFIG2     0x02
#define ADS_ADR_CONFIG3     0x03
#define ADS_ADR_LOFF        0x04
#define ADS_ADR_CH1SET      0x05
#define ADS_ADR_CH2SET      0x06
#define ADS_ADR_CH3SET      0x07
#define ADS_ADR_CH4SET      0x08
#define ADS_ADR_CH5SET      0x09
#define ADS_ADR_CH6SET      0x0A
#define ADS_ADR_CH7SET      0x0B
#define ADS_ADR_CH8SET      0x0C
#define ADS_ADR_BIAS_SENSP  0x0D
#define ADS_ADR_BIAS_SENSN  0x0E
#define ADS_ADR_LOFF_SENSP  0x0F
#define ADS_ADR_LOFF_SENSN  0x10
#define ADS_ADR_LOFF_FLIP   0x11
#define ADS_ADR_LOFF_STATP  0x12
#define ADS_ADR_LOFF_STATN  0x13
#define ADS_ADR_GPIO        0x14
#define ADS_ADR_MISC1       0x15
#define ADS_ADR_MISC2       0x16
#define ADS_ADR_CONFIG4     0x17
#define ADS_ADR_WCT1        0x18
#define ADS_ADR_WCT2        0x19

// CONFIG1 REGISTER begin

/** @brief High-resolution or low-power mode
  * This bit determines whether the device runs in low-power or
  * high-resolution mode.
  */
typedef enum
{
  CONFIG1_HR_LowPower       = 0,
  CONFIG1_HR_HighResolution = 1
}ADS_CONFIG1_HR_Bit;

/** @brief Daisy-chain or multiple readback mode.
  * This bit determines which mode is enabled
  */
typedef enum
{
  CONFIG1_DAISY_DaisyChain       = 0,
  CONFIG1_DAISY_MultipleReadback = 1
}ADS_CONFIG1_DAISY_EN_Bit;

/** @brief CLK connection.
  * This bit determines if the internal
  * oscillator signal is connected to
  * the CLK pin when the CLKSEL pin = 1
  */
typedef enum
{
  CONFIG1_CLK_OscClkOutDisable = 0,
  CONFIG1_CLK_OscClkOutEnable  = 1
}ADS_CONFIG1_CLK_EN_Bit;

/** @brief Output data rate.3 bits
  * For High-Resolution mode, f MOD = f CLK / 4.
  * For low power mode, f MOD = f CLK / 8.
  * These bits determine the output data
  * rate of the device
  */
typedef enum
{
  CONFIG1_DR_Fmod_Div_16   = 0b000 ,
  CONFIG1_DR_Fmod_Div_32   = 0b001 ,
  CONFIG1_DR_Fmod_Div_64   = 0b010 ,
  CONFIG1_DR_Fmod_Div_128  = 0b011 ,
  CONFIG1_DR_Fmod_Div_256  = 0b100 ,
  CONFIG1_DR_Fmod_Div_512  = 0b101 ,
  CONFIG1_DR_Fmod_Div_1024 = 0b110 ,
}ADS_CONFIG1_DR_Bits;


/** @brief Configuration Register 1
  */
typedef union
{
    struct
    {
        ADS_CONFIG1_DR_Bits        DR:       3;
        unsigned int               RESERVED: 2;
        ADS_CONFIG1_CLK_EN_Bit     CLK_EN:   1;
        ADS_CONFIG1_DAISY_EN_Bit   DAISY_EN: 1;
        ADS_CONFIG1_HR_Bit         HR:       1;
    } s;
    uint8_t byte;
}ADS_CONFIG1_InitTypeDef;


// CONFIG1 REGISTER end

// CONFIG2 REGISTER begin

/** @brief WCT chopping scheme.
  * This bit determines whether the chopping
  * frequency of WCT amplifiers is variable or fixed
  */
typedef enum
{
  CONFIG2_WCT_CHOP_ChoppingFreqVaries = 0,
  CONFIG2_WCT_CHOP_ChoppingFreqConst = 1 // Fmod / 16
}ADS_CONFIG2_WCT_CHOP_Bit;


/** @brief TEST source.
  * This bit determines the source for the
  * test signal
  */
typedef enum
{
  CONFIG2_INT_TEST_TestSignalsExternal = 0,
  CONFIG2_INT_TEST_TestSignalsInternal = 1
}ADS_CONFIG2_INT_TEST_Bit;

/** @brief Test signal amplitude.
  * These bits determine the calibration
  * signal amplitude
  */
typedef enum
{
  /// @brief 1 * -(VREFP - VREFN) / 2400 V
  CONFIG2_TEST_AMP_TestSignalsAmp_1_DIV_2400V = 0,
  /// @brief 2 * -(VREFP - VREFN) / 2400 V
  CONFIG2_TEST_AMP_TestSignalsAmp_2_DIV_2400V = 1
}ADS_CONFIG2_TEST_AMP_Bit;


/** @brief Test signal frequency.
  * These bits determine the calibration
  * signal frequency
  */
typedef enum
{
  CONFIG2_TEST_FREQ_Fclk_DIV_2pow21   = 0b00,
  CONFIG2_TEST_FREQ_Fclk_DIV_2pow20   = 0b01,
  CONFIG2_TEST_FREQ_NotUsed           = 0b10,
  CONFIG2_TEST_FREQ_DC                = 0b11,
}ADS_CONFIG2_TEST_FREQ_Bits;

/** @brief Configuration Register 2
 */
typedef union
{
    struct
    {
        ADS_CONFIG2_TEST_FREQ_Bits TEST_FREQ:2;
        ADS_CONFIG2_TEST_AMP_Bit   TEST_AMP: 1;
        unsigned int               RESERVED2:1;
        ADS_CONFIG2_INT_TEST_Bit   INT_TEST: 1;
        ADS_CONFIG2_WCT_CHOP_Bit   WCT_CHOP: 1;
        unsigned int               RESERVED1:2;
    } s;
    uint8_t byte;
}ADS_CONFIG2_InitTypeDef;

// CONFIG2 REGISTER end

// CONFIG3 REGISTER enums begin



typedef enum
{
  CONFIG_PD_REFBUF_PowerDown = 0,
  CONFIG_PD_REFBUF_Enable    = 1
}ADS_CONFIG_PD_REFBUF_Bit;



typedef enum
{
  CONFIG_VREF_2dot4V = 0,
  CONFIG_VREF_4V     = 1
}ADS_CONFIG_VREF_4V_Bit;



typedef enum
{
  CONFIG_RLD_MEAS_Open   = 0,
  CONFIG_RLD_MEAS_Routed = 1
}ADS_CONFIG_RLD_MEAS_Bit;


typedef enum
{
  CONFIG_RLDREF_External = 0,
  /// @brief RLDREF signal (AVDD - AVSS) / 2 generated internally
  CONFIG_RLDREF_Internal = 1
}ADS_CONFIG_RLDREF_INT_Bit;



typedef enum
{
  CONFIG_PD_RLD_PowerDown = 0,
  CONFIG_PD_RLD_Enable = 1
}ADS_CONFIG_PD_RLD_Bit;



typedef enum
{
  CONFIG_RLD_LOFF_SENS_Disable = 0,
  CONFIG_RLD_LOFF_SENS_Enable  = 1
}ADS_CONFIG_RLD_LOFF_SENS_Bit;



typedef enum
{
  CONFIG_RLD_STAT_Connected    = 0,
  CONFIG_RLD_STAT_NotConnected = 1
}ADS_CONFIG_RLD_STAT_Bit;

// CONFIG3 REGISTER enums end



typedef struct
{

  //CONFIG3 Register
  ADS_CONFIG_PD_REFBUF_Bit  PD_REFBUF; /*!< Power-down reference buffer.
                           	   	   	   This bit determines the power-down
                           	   	   	   reference buffer state */

  ADS_CONFIG_VREF_4V_Bit    VREF_4V;   /*!< Reference voltage.
                           	   	   	   This bit determines the reference
                           	   	   	   voltage, VREFP */

  ADS_CONFIG_RLD_MEAS_Bit   RLD_MEAS;  /*!< Reference voltage.
                           	   	   	   This bit determines the reference
                           	   	   	   voltage, VREFP */

  ADS_CONFIG_RLDREF_INT_Bit RLDREF_INT;/*!< RLDREF signal.
                           	   	   	   This bit determines the RLDREF signal
                           	   	   	   source */

  ADS_CONFIG_PD_RLD_Bit     PD_RLD;    /*!< RLD buffer power.
                           	   	   	   This bit determines the RLD buffer
                           	   	   	   power state */

  ADS_CONFIG_RLD_LOFF_SENS_Bit RLD_LOFF_SENS;  /*!< RLD sense function.
                           	   	   	   This bit enables the RLD sense function */

  ADS_CONFIG_RLD_STAT_Bit   RLD_STAT;  /*!< RLD lead-off status.
                           	   	   	   This bit determines the RLD status */

}ADS_CONFIG_InitTypeDef;







extern uint8_t verbose;

extern int32_t channelData [8];

void ADS_Init(void);

void ADS_RDATAC(void);
void ADS_SDATAC(void);
void ADS_RESET(void);
void ADS_START(void);
void ADS_STOP(void);
void ADS_WAKEUP(void);

uint8_t ADS_getDeviceID(void);
uint8_t ADS_RREG(uint8_t _address);

void ADS_RREGS(uint8_t _address, uint8_t _numRegistersMinusOne);
void ADS_WREG(uint8_t _address, uint8_t _value);
void ADS_WREGS(uint8_t _address, uint8_t _numRegistersMinusOne);
void ADS_updateChannelData(void);
void ADS_RDATA(void);
void ADS_printRegisterName(uint8_t _address);


void ADS_SendData();

extern uint8_t transferSPI(uint8_t send);
extern void USART_Send(char* string, uint16_t size);

#endif //__ADS1298_H
