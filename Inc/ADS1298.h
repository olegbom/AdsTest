
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
#define ADS_ADR_RLD_SENSP   0x0D
#define ADS_ADR_RLD_SENSN   0x0E
#define ADS_ADR_LOFF_SENSP  0x0F
#define ADS_ADR_LOFF_SENSN  0x10
#define ADS_ADR_LOFF_FLIP   0x11
#define ADS_ADR_LOFF_STATP  0x12
#define ADS_ADR_LOFF_STATN  0x13
#define ADS_ADR_GPIO        0x14
#define ADS_ADR_PACE        0x15
#define ADS_ADR_RESP        0x16
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

// CONFIG3 REGISTER begin

/** @brief Power-down reference buffer.
  * This bit determines the power-down
  * reference buffer state
  */
typedef enum
{
  CONFIG3_PD_REFBUF_PowerDown = 0,
  CONFIG3_PD_REFBUF_Enable    = 1
}ADS_CONFIG3_PD_REFBUF_Bit;


/** @brief Reference voltage.
  * This bit determines the reference
  * voltage, VREFP
  */
typedef enum
{
  CONFIG3_VREF_2dot4V = 0,
  CONFIG3_VREF_4V     = 1
}ADS_CONFIG3_VREF_4V_Bit;


/** @brief RLD measurement.
  * This bit enables RLD measurement.
  * The RLD signal may be measured
  * with any channel
  */
typedef enum
{
  CONFIG3_RLD_MEAS_Open   = 0,
  CONFIG3_RLD_MEAS_Routed = 1
}ADS_CONFIG3_RLD_MEAS_Bit;

/** @brief RLDREF signal.
  * This bit determines the RLDREF signal
  * source
  */
typedef enum
{
  CONFIG3_RLDREF_External = 0,
  /// @brief RLDREF signal (AVDD - AVSS) / 2 generated internally
  CONFIG3_RLDREF_Internal = 1
}ADS_CONFIG3_RLDREF_INT_Bit;


/** @brief RLD buffer power.
  * This bit determines the RLD buffer
  * power state
  */
typedef enum
{
  CONFIG3_PD_RLD_PowerDown = 0,
  CONFIG3_PD_RLD_Enable = 1
}ADS_CONFIG3_PD_RLD_Bit;


/** @brief RLD sense function.
  * This bit enables the RLD sense function
  */
typedef enum
{
  CONFIG3_RLD_LOFF_SENS_Disable = 0,
  CONFIG3_RLD_LOFF_SENS_Enable  = 1
}ADS_CONFIG3_RLD_LOFF_SENS_Bit;


/** @brief RLD lead-off status.
  * This bit determines the RLD status
  */
typedef enum
{
  CONFIG3_RLD_STAT_Connected    = 0,
  CONFIG3_RLD_STAT_NotConnected = 1
}ADS_CONFIG3_RLD_STAT_Bit;

/** @brief Configuration Register 3
  */
typedef union
{
    struct
    {
        ADS_CONFIG3_RLD_STAT_Bit      RLD_STAT      :1;
        ADS_CONFIG3_RLD_LOFF_SENS_Bit RLD_LOFF_SENS :1;
        ADS_CONFIG3_PD_RLD_Bit        PD_RLD        :1;
        ADS_CONFIG3_RLDREF_INT_Bit    RLDREF_INT    :1;
        ADS_CONFIG3_RLD_MEAS_Bit      RLD_MEAS      :1;
        ADS_CONFIG3_VREF_4V_Bit       VREF_4V       :1;
        unsigned int                  RESERVED      :1;
        ADS_CONFIG3_PD_REFBUF_Bit     PD_REFBUF     :1;
    } s;
    uint8_t byte;
}ADS_CONFIG3_InitTypeDef;

// CONFIG3 REGISTER end

// LOFF: Lead-Off Control Register start

/** @brief Lead-off comparator threshold
  */
typedef enum
{
  LOFF_COMP_TH_P_95_N_5      = 0b000,
  LOFF_COMP_TH_P_92_5_N_7_5  = 0b001,
  LOFF_COMP_TH_P_90_N_10     = 0b010,
  LOFF_COMP_TH_P_87_5_N_12_5 = 0b011,
  LOFF_COMP_TH_P_85_N_15     = 0b100,
  LOFF_COMP_TH_P_80_5_N_20   = 0b101,
  LOFF_COMP_TH_P_75_N_25     = 0b110,
  LOFF_COMP_TH_P_70_N_30     = 0b111
}ADS_LOFF_COMP_TH_Bits;

/** @brief Lead-off detection mode
  * This bit determines the lead-off detection mode
  */
typedef enum
{
  LOFF_VLEAD_OFF_EN_CurrentSource = 0,
  LOFF_VLEAD_OFF_EN_PullResistor  = 1,
}ADS_LOFF_VLEAD_OFF_EN_Bit;

/** @brief Lead-off current magnitude
  * These bits determine the magnitude of
  * current for the current lead-off mode
  */
typedef enum
{
  LOFF_ILEAD_OFF_6_nA  = 0b00,
  LOFF_ILEAD_OFF_12_nA = 0b01,
  LOFF_ILEAD_OFF_18_nA = 0b10,
  LOFF_ILEAD_OFF_24_nA = 0b11,
}ADS_LOFF_ILEAD_OFF_Bits;

/** @brief Lead-off frequency
  * These bits determine the frequency of
  * lead-off detect for each channel
  */
typedef enum
{
  LOFF_FLEAD_OFF_TurnOff         = 0b00,
  LOFF_FLEAD_OFF_AC_at_fDR_DIV_4 = 0b01,
  LOFF_FLEAD_OFF_DC              = 0b11,
}ADS_LOFF_FLEAD_OFF_Bits;

/** @brief Lead-Off Control Register
  */
typedef union
{
    struct
    {
        ADS_LOFF_FLEAD_OFF_Bits   FLEAD_OFF    :2;
        ADS_LOFF_ILEAD_OFF_Bits   ILEAD_OFF    :2;
        ADS_LOFF_VLEAD_OFF_EN_Bit VLEAD_OFF_EN :1;
        ADS_LOFF_COMP_TH_Bits     COMP_TH      :3;
    } s;
    uint8_t byte;
}ADS_LOFF_InitTypeDef;

// LOFF: Lead-Off Control Register end

// CHnSET: Individual Channel Settings begin

/** @brief Power-down
  * This bit determines the channel
  * power mode for the
  * corresponding channel.
  */
typedef enum
{
  CHnSET_PD_Normal = 0,
  /// @brief See datasheet
  CHnSET_PD_PowerDown = 1,
}ADS_CHnSET_PD_Bit;

/** @brief PGA gain
  * These bits determine the PGA gain setting
  */
typedef enum
{
  CHnSET_GAIN_6  = 0b000,
  CHnSET_GAIN_1  = 0b001,
  CHnSET_GAIN_2  = 0b010,
  CHnSET_GAIN_3  = 0b011,
  CHnSET_GAIN_4  = 0b100,
  CHnSET_GAIN_8  = 0b101,
  CHnSET_GAIN_12 = 0b110,
}ADS_CHnSET_GAIN_Bits;

/** @brief Channel input
  * These bits determine the channel input selection
  */
typedef enum
{
  CHnSET_MUX_NormalInput  = 0b000,
  CHnSET_MUX_InputShorted = 0b001,
  CHnSET_MUX_RLD_MEAS     = 0b010,
  CHnSET_MUX_MVDD         = 0b011,
  CHnSET_MUX_TempSensor   = 0b100,
  CHnSET_MUX_TestSignal   = 0b101,
  CHnSET_MUX_RLD_DRP      = 0b110,
  CHnSET_MUX_RLD_DRN      = 0b111,
}ADS_CHnSET_MUX_Bits;


/** @brief Individual Channel Settings
  */
typedef union
{
    struct
    {
        ADS_CHnSET_MUX_Bits  MUX      :3;
        unsigned int         RESERVED :1;
        ADS_CHnSET_GAIN_Bits GAIN     :3;
        ADS_CHnSET_PD_Bit    PD       :1;

    } s;
    uint8_t byte;
}ADS_CHnSET_InitTypeDef;

// CHnSET: Individual Channel Settings end

// RLD_SENSP: RLD Positive Signal Derivation Register begin

/** @brief RLD Positive Signal Derivation Register
  */
typedef union
{
    struct
    {
        FunctionalState RLD1P:1;
        FunctionalState RLD2P:1;
        FunctionalState RLD3P:1;
        FunctionalState RLD4P:1;
        FunctionalState RLD5P:1;
        FunctionalState RLD6P:1;
        FunctionalState RLD7P:1;
        FunctionalState RLD8P:1;
    } s;
    uint8_t byte;
}ADS_RLD_SENSP_InitTypeDef;

// RLD_SENSP: RLD Positive Signal Derivation Register end

// RLD_SENSN: RLD Negative Signal Derivation Register begin

/** @brief RLD Negative Signal Derivation Register
  */
typedef union
{
    struct
    {
        FunctionalState RLD1N:1;
        FunctionalState RLD2N:1;
        FunctionalState RLD3N:1;
        FunctionalState RLD4N:1;
        FunctionalState RLD5N:1;
        FunctionalState RLD6N:1;
        FunctionalState RLD7N:1;
        FunctionalState RLD8N:1;
    } s;
    uint8_t byte;
}ADS_RLD_SENSN_InitTypeDef;

// RLD_SENSN: RLD Negative Signal Derivation RegisterA end

// LOFF_SENSP: Positive Signal Lead-Off Detection Register  begin

/** @brief RLD Positive Signal Lead-Off Detection Register
  */
typedef union
{
    struct
    {
        FunctionalState LOFF1P:1;
        FunctionalState LOFF2P:1;
        FunctionalState LOFF3P:1;
        FunctionalState LOFF4P:1;
        FunctionalState LOFF5P:1;
        FunctionalState LOFF6P:1;
        FunctionalState LOFF7P:1;
        FunctionalState LOFF8P:1;
    } s;
    uint8_t byte;
}ADS_LOFF_SENSP_InitTypeDef;

// LOFF_SENSP: Positive Signal Lead-Off Detection Register  end

// LOFF_SENSN: Negative Signal Lead-Off Detection Register  begin

/** @brief RLD Negative Signal Lead-Off Detection Register
  */
typedef union
{
    struct
    {
        FunctionalState LOFF1N:1;
        FunctionalState LOFF2N:1;
        FunctionalState LOFF3N:1;
        FunctionalState LOFF4N:1;
        FunctionalState LOFF5N:1;
        FunctionalState LOFF6N:1;
        FunctionalState LOFF7N:1;
        FunctionalState LOFF8N:1;
    } s;
    uint8_t byte;
}ADS_LOFF_SENSN_InitTypeDef;

// LOFF_SENSN: Negative Signal Lead-Off Detection Register end

// LOFF_FLIP: Lead-Off Flip Register begin

/** @brief Channel LOFF polarity flip
  * Flip the pullup/pulldown polarity
  * of the current source or resistor
  * on channel for lead-off derivation
  */
typedef enum
{
    LOFF_FLIP_NoFilp  = 0,
    LOFF_FLIP_Flipped = 1,
}ADS_LOFF_FLIP_Bit;
/** @brief Lead-Off Flip Register
  * This register controls the direction
  * of the current used for lead-off derivation
  */
typedef union
{
    struct
    {
        ADS_LOFF_FLIP_Bit LOFF_FLIP1:1;
        ADS_LOFF_FLIP_Bit LOFF_FLIP2:1;
        ADS_LOFF_FLIP_Bit LOFF_FLIP3:1;
        ADS_LOFF_FLIP_Bit LOFF_FLIP4:1;
        ADS_LOFF_FLIP_Bit LOFF_FLIP5:1;
        ADS_LOFF_FLIP_Bit LOFF_FLIP6:1;
        ADS_LOFF_FLIP_Bit LOFF_FLIP7:1;
        ADS_LOFF_FLIP_Bit LOFF_FLIP8:1;
    } s;
    uint8_t byte;
}ADS_LOFF_FLIP_InitTypeDef;

// LOFF_FLIP: Lead-Off Flip Register end

// LOFF_STATP: Lead-Off Positive Signal Status Register  begin

/** @brief Channel lead-off status
  * Status of whether electrode is on or off
  */
typedef enum
{
    LOFF_STAT_On  = 0,
    LOFF_STAT_Off = 1,
}ADC_LOFF_STAT_Bit;

/** @brief Lead-Off Positive Signal Status Register
  * This register stores the status of whether the positive
  * electrode on each channel is on or off
  */
typedef union
{
    struct
    {
        ADC_LOFF_STAT_Bit IN1P_OFF:1;
        ADC_LOFF_STAT_Bit IN2P_OFF:1;
        ADC_LOFF_STAT_Bit IN3P_OFF:1;
        ADC_LOFF_STAT_Bit IN4P_OFF:1;
        ADC_LOFF_STAT_Bit IN5P_OFF:1;
        ADC_LOFF_STAT_Bit IN6P_OFF:1;
        ADC_LOFF_STAT_Bit IN7P_OFF:1;
        ADC_LOFF_STAT_Bit IN8P_OFF:1;
    } s;
    uint8_t byte;
}ADS_LOFF_STATP_InitTypeDef;

// LOFF_STATP: Lead-Off Positive Signal Status Register  end

// LOFF_STATN: Lead-Off Negative Signal Status Register   begin

/** @brief RLD Lead-Off Negative Signal Status Register
  * This register stores the status of whether the negative
  * electrode on each channel is on or off
  */
typedef union
{
    struct
    {
        ADC_LOFF_STAT_Bit IN1N_OFF:1;
        ADC_LOFF_STAT_Bit IN2N_OFF:1;
        ADC_LOFF_STAT_Bit IN3N_OFF:1;
        ADC_LOFF_STAT_Bit IN4N_OFF:1;
        ADC_LOFF_STAT_Bit IN5N_OFF:1;
        ADC_LOFF_STAT_Bit IN6N_OFF:1;
        ADC_LOFF_STAT_Bit IN7N_OFF:1;
        ADC_LOFF_STAT_Bit IN8N_OFF:1;
    } s;
    uint8_t byte;
}ADS_LOFF_STATN_InitTypeDef;

// LOFF_STATN: Lead-Off Negative Signal Status Register  end

// GPIO: General-Purpose I/O Register begin

/** @brief GPIO control
  * These bits determine if the corresponding
  * GPIOD pin is an input or output
  */
typedef enum
{
    GPIOC_Output  = 0,
    GPIOC_Input = 1,
}ADC_GPIOC_Bit;

typedef union
{
    struct
    {
        ADC_GPIOC_Bit GPIOC1:1;
        ADC_GPIOC_Bit GPIOC2:1;
        ADC_GPIOC_Bit GPIOC3:1;
        ADC_GPIOC_Bit GPIOC4:1;
        FlagStatus    GPIOD1:1;
        FlagStatus    GPIOD2:1;
        FlagStatus    GPIOD3:1;
        FlagStatus    GPIOD4:1;
    } s;
    uint8_t byte;
}ADS_GPIO_InitTypeDef;

// GPIO: General-Purpose I/O Register end


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
