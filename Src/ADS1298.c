/*------------------------------- DEFINES-----------------------------*/
#include "ADS1298.h"

/*------------------------------- VARIABLES -----------------------------*/

int stat_1;    	// used to hold the status register for boards 1 and 2

uint8_t regData[24];		// array with data from all registers
int32_t channelData [8];	// array used when reading channel data board 1+2

/*------------------------------- FUNCTIIONS -----------------------------*/

void ADS_Init(){
	

	ADS_RESET();
	HAL_Delay(500);
	
	ADS_SDATAC();
	HAL_Delay(500);
	
	ADS_getDeviceID();
	HAL_Delay(1000);

	//Work settings

	ADS_CONFIG1_InitTypeDef config1 = {.byte = 0x00};
	config1.s.DR = CONFIG1_DR_Fmod_Div_1024;
	config1.s.HR = CONFIG1_HR_LowPower;
	config1.s.CLK_EN = CONFIG1_CLK_OscClkOutDisable;
	config1.s.DAISY_EN = CONFIG1_DAISY_DaisyChain;
	ADS_WREG(ADS_ADR_CONFIG1, config1.byte);


	//0b0000_0110
	//7 bit on 0 - High-resolution(1) or low-power(0) mode
	//6 bit on 0 - Daisy-chain(0) or multiple readback(1) mode
	//5 bit on 0 - CLK connection:
	//  Oscillator clock output disabled(0) or enabled(1)
	//4:3 bit on 0 - Reserved
	//2:0 bit on 100 - Output data rate 100: f MOD / 1024 (1 kSPS)
	//ADS_WREG(ADS_ADR_CONFIG1,0x06);
	HAL_Delay(100);


    ADS_CONFIG2_InitTypeDef config2 = {.byte = 0x00};
    config2.s.INT_TEST = CONFIG2_INT_TEST_TestSignalsInternal;
    config2.s.TEST_AMP = CONFIG2_TEST_AMP_TestSignalsAmp_1_DIV_2400V;
    config2.s.TEST_FREQ = CONFIG2_TEST_FREQ_Fclk_DIV_2pow20;
    config2.s.WCT_CHOP = CONFIG2_WCT_CHOP_ChoppingFreqVaries;
    ADS_WREG(ADS_ADR_CONFIG2, config2.byte);

	//0b0001_0001
	//7:6 bit on 0 - Reserved
	//5 bit on 0 - WCT chopping scheme, varies(0) or constant f MOD / 16 (1)
	//4 bit on 1 - TEST source - externally(0) or internally(1)
	//3 bit on 0 - Reserved
	//2 bit on 0 - Test signal amplitude
	//1:0 bit on 00 -  Pulsed at f CLK / 2^21
	//ADS_WREG(ADS_ADR_CONFIG2,0x11);
	HAL_Delay(100);

	ADS_CONFIG3_InitTypeDef config3 = {.byte = 0x00};
	config3.s.PD_REFBUF = CONFIG3_PD_REFBUF_Enable;
	config3.s.VREF_4V = CONFIG3_VREF_4V;
	config3.s.RLD_MEAS = CONFIG3_RLD_MEAS_Routed;
	config3.s.RLDREF_INT = CONFIG3_RLDREF_Internal;
	config3.s.PD_RLD = CONFIG3_PD_RLD_Enable;
	config3.s.RLD_LOFF_SENS = CONFIG3_RLD_LOFF_SENS_Disable;
	config3.s.RLD_STAT = CONFIG3_RLD_STAT_Connected;
	ADS_WREG(ADS_ADR_CONFIG3, config3.byte);
	//0b1101_1100
	//7 bit on 1 - Power-down reference buffer: Enable(1) or Power-down(0)
	//6 bit on 1 - Reserved
	//5 bit on 0 - Reference voltage: 2.4V(0) or 4V(1)
	//4 bit on 1 - RLD measurement: Open(0) or routed MUX(1)
	//3 bit on 1 - RLDREF signal: externally(0) or internally(1)
	//2 bit on 1 - RLD buffer power: Enable(1) or Power-down(0)
	//1 bit on 0 - RLD sense function: Enable(1) or Disabled(0)
	//0 bit on 0 - RLD lead-off status: Connected(0) or not connected(1)
	//ADS_WREG(ADS_ADR_CONFIG3,0xDC);
	HAL_Delay(100);
	
	ADS_LOFF_InitTypeDef loff = {.byte = 0x00};
	loff.s.COMP_TH = LOFF_COMP_TH_P_95_N_5;
	loff.s.VLEAD_OFF_EN = LOFF_VLEAD_OFF_EN_CURRENT_SOURCE;
	loff.s.ILEAD_OFF = LOFF_ILEAD_OFF_6_nA;
	loff.s.FLEAD_OFF = LOFF_FLEAD_OFF_DC;
	ADS_WREG(ADS_ADR_LOFF, loff.byte);
	//0b0000_0011
	//7:5 bit on 000 - Lead-off comparator threshold: 000 = 95%
	//4 bit on 0 - Lead-off detection mode:
	//  0 - Current source mode lead-off
	//  1 - pullup or pulldown resistor mode lead-off
	//3:2 bit on 00 - Lead-off current magnitude: 00 = 6 nA
	//1:0 bit on 11 - Lead-off frequency: 11 = DC lead-off detection turned on
	//ADS_WREG(ADS_ADR_LOFF,0x03);
	HAL_Delay(10);
	
	//0b0101_0000
	//7 bit on 0 - Power-down: On(0) or Off(1)
	//6:4 bit on 101 - PGA gain: 101 = 8
	//3 bit on 0 - Reserved
	//2:0 bit on 000 - Channel input: 000 = Normal electrode input
	ADS_WREG(ADS_ADR_CH1SET,0x50);  
	HAL_Delay(10);
	ADS_WREG(ADS_ADR_CH2SET,0x50);  
	HAL_Delay(10);
	ADS_WREG(ADS_ADR_CH3SET,0x50);  
	HAL_Delay(10);
	ADS_WREG(ADS_ADR_CH4SET,0x50);  
	HAL_Delay(10);
	ADS_WREG(ADS_ADR_CH5SET,0x50);
	HAL_Delay(10);
	ADS_WREG(ADS_ADR_CH6SET,0x50);
	HAL_Delay(10);
	ADS_WREG(ADS_ADR_CH7SET,0x50);
	HAL_Delay(10);
	ADS_WREG(ADS_ADR_CH8SET,0x55);
	HAL_Delay(10);
	

	//0b0000_0000
	//7:0 bit on 0 - INxP to RLD: Disabled (0) or Enabled (1)
	ADS_WREG(ADS_ADR_BIAS_SENSP,0x00);  
	HAL_Delay(10);

	//0b0000_0000
	//7:0 bit on 0 - INxN to RLD: Disabled (0) or Enabled (1)
	ADS_WREG(ADS_ADR_BIAS_SENSN,0x00);  
	HAL_Delay(10);
	
	//0b1111_1111
	//7:0 bit on 0 - INxP lead off: Disabled (0) or Enabled (1)
	ADS_WREG(ADS_ADR_LOFF_SENSP,0xFF);  
	HAL_Delay(10);

	//0b0000_0010
	//7:0 bit on 0 - INxN lead off: Disabled (0) or Enabled (1)
	ADS_WREG(ADS_ADR_LOFF_SENSN,0x00);
	HAL_Delay(10);
	
	//0b0000_0000
	//7:0 bit on 0 - Channel X LOFF polarity flip: No Flip (0) or Flipped (1)
	ADS_WREG(ADS_ADR_LOFF_FLIP,0x00);  
	HAL_Delay(10);
	
	//0b1111_1000
	//7:0 bit on 0 - Channel X positive channel lead-off status: On (0) or Off (1)
	ADS_WREG(ADS_ADR_LOFF_STATP,0x00);
	HAL_Delay(10);
	
	//0b1111_1000
	//7:0 bit on 0 - Channel X negative channel lead-off status: On(0) or Off(1)
	ADS_WREG(ADS_ADR_LOFF_STATN,0x00);  
	HAL_Delay(10);
	
	//0b0000_0000
	//7:4 bit on 0 - GPIO data
	//3:0 bit on 0 - GPIO control: Output(0) or Input(1)
	ADS_WREG(ADS_ADR_GPIO,0x00);  
	HAL_Delay(10);
	
	//0b0000_0000
	//7:5 bit on 000 - Reserved
	//4:3 bit on 00 - Pace even channels: 00 = Channel 2
	//2:1 bit on 00 - Pace odd channels: 00 = Channel 1
	//0 bit on 0 - Pace detect buffer: Off(0) or On(1)
	ADS_WREG(ADS_ADR_MISC1,0x00);  
	HAL_Delay(10);

	//0b1111_0000
	//7 bit on 1 - Respiration demodulation circuitry: Off(0) or On(1)
	//6 bit on 1 - Respiration modulation circuitry: Off(0) or On(1)
	//5 bit on 1 - Reserved
	//4:2 bit on 100 - Respiration phase: 100 = 112.5deg
	//1:0 bit on 00 - Respiration control: 00 = No respiration
	ADS_WREG(ADS_ADR_MISC2,0xF0);  
	HAL_Delay(10);
	
	//0b0010_0010
	//7:5 bit on 001 - Respiration modulation frequency: 001 = 32 kHz modulation clock
	//4 bit on 0 - Reserved
	//3 bit on 0 - Single-shot conversion: Continuous(0) or Single-shot(1)
	//2 bit on 0 - Connects the WCT to the RLD: Off(0) or On(1)
	//1 bit on 1 - Lead-off comparator power-down: Disabled(0) or Enabled(1)
	//0 bit on 0 - Reserved
	ADS_WREG(ADS_ADR_CONFIG4,0x22);  
	HAL_Delay(10);
	

	//0b0000_1010
	//7 bit on 0 - Enable (WCTA + WCTB)/2 to the negative input of channel 6: Disabled(0) or Enabled(1)
	//6 bit on 0 - Enable (WCTA + WCTC)/2 to the negative input of channel 5: Disabled(0) or Enabled(1)
	//5 bit on 0 - Enable (WCTB + WCTC)/2 to the negative input of channel 7: Disabled(0) or Enabled(1)
	//4 bit on 0 - Enable (WCTB + WCTC)/2 to the negative input of channel 4: Disabled(0) or Enabled(1)
	//3 bit on 1 - Power-down WCTA: Down(0) or On(1)
	//2:0 bit on 010 - WCT Amplifier A channel selection: 010 = Channel 2 positive input connected to WCTA amplifier
	ADS_WREG(ADS_ADR_WCT1,0x0A);
	HAL_Delay(10);
	
	//0b1110_0011
	//7 bit on 1 - Power-down WCTC: Down(0) or On(1)
	//6 bit on 1 - Power-down WCTB: Down(0) or On(1)
	//5:3 bit on 100 - WCT amplifier B channel selection: 100 = Channel 3 positive input connected
	//2:0 bit on 011 - WCT amplifier C channel selection: 011 = Channel 2 negative input connected
	ADS_WREG(ADS_ADR_WCT2,0xE3);
	HAL_Delay(10);
	

	ADS_RREGS(0,17);
	HAL_Delay(1000);
	
	//ADS_START();
	HAL_Delay(100);

	ADS_RDATAC();            // enter Read Data Continuous mode
	HAL_Delay(100);

	
	

}

#define ADC_CS_ENABLE() HAL_GPIO_WritePin( ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET)
#define ADC_CS_DISABLE() HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);

void ADS_WriteOpcode(uint8_t opcode)
{
   // ADC_CS_ENABLE();
    transferSPI(ADS_OPCODE_SDATAC);
   // ADC_CS_DISABLE();
}

void ADS_SDATAC(){
    ADS_WriteOpcode(ADS_OPCODE_SDATAC);
}

//start data conversion 
void ADS_START() {
    ADS_WriteOpcode(ADS_OPCODE_START);

}

//stop data conversion 
void ADS_STOP() {			
    ADS_WriteOpcode(ADS_OPCODE_STOP);
}

void ADS_RESET(){
    ADS_WriteOpcode(ADS_OPCODE_RESET);

}

void ADS_RDATAC() {
    ADS_WriteOpcode(ADS_OPCODE_RDATAC);

}

void ADS_WAKEUP() {
    ADS_WriteOpcode(ADS_OPCODE_WAKEUP);
}

// Register Read/Write Commands
uint8_t ADS_getDeviceID() {			// simple hello world com check
	
	uint8_t data = ADS_RREG(0);

	
	return data;
}

uint8_t ADS_RREG(uint8_t _address){		//  reads ONE register at _address
	uint8_t opcode1 = _address + 0x20; 		//  RREG expects 001rrrrr where rrrrr = _address
	
	transferSPI( opcode1); 								//  opcode1
    transferSPI( 0); 											//  opcode2
  
	regData[_address] = transferSPI( 0);		//  update mirror location with returned byte
	
	
	//HAL_UART_Transmit(&huart1, &regData[0], sizeof(uint8_t),0x1000);
	
    return 0;
}

// Read more than one register starting at _address
void ADS_RREGS(uint8_t _address, uint8_t _numRegistersMinusOne) {
//	for(byte i = 0; i < 0x17; i++){
//		regData[i] = 0;									//  reset the regData array
//	}
	int i;
	
	uint8_t opcode1 = _address + 0x20; 				//  RREG expects 001rrrrr where rrrrr = _address
	
	transferSPI( opcode1); 										//  opcode1
	transferSPI( _numRegistersMinusOne);				//  opcode2

	for(i = 0; i <= _numRegistersMinusOne; i++){
		regData[_address + i] = transferSPI( 0x00); 	//  add register byte to mirror array
	}
	
	

}


void ADS_WREG(uint8_t _address, uint8_t _value) {	//  Write ONE register at _address
	uint8_t opcode1 = _address + 0x40; 				//  WREG expects 010rrrrr where rrrrr = _address
	
	
	transferSPI( opcode1);											//  Send WREG command & address
	transferSPI( 0x00);												//	Send number of registers to read -1
	transferSPI( _value);											//  Write the value to the register
	
	
	regData[_address] = _value;			//  update the mirror array
	

}

void ADS_WREGS(uint8_t _address, uint8_t _numRegistersMinusOne){
	int i;
	
	uint8_t opcode1 = _address + 0x40;				//  WREG expects 010rrrrr where rrrrr = _address
	
	
	transferSPI( opcode1);											//  Send WREG command & address
	transferSPI( _numRegistersMinusOne);				//	Send number of registers to read -1	
	
	for (i=_address; i <=(_address + _numRegistersMinusOne); i++){
		transferSPI( regData[i]);								//  Write to the registers
	}	
	
	

}

void ADS_updateChannelData(){
	uint8_t inByte;
	int i,j;				// iterator in loop
	int nchan=8;  //assume 8 channel.  If needed, it automatically changes to 16 automatically in a later block.
	
	
	for(i = 0; i < nchan; i++){
		channelData[i] = 0;
	}
	
	
	// READ CHANNEL DATA FROM FIRST ADS IN DAISY LINE
	for(i = 0; i < 3; i++){										//  read 3 byte status register from ADS 1 (1100+LOFF_STATP+LOFF_STATN+GPIO[7:4])
		inByte = transferSPI( 0x00);
		stat_1 = (stat_1<<8) | inByte;				
	}
	
	for(i = 0; i < 8; i++){
		for( j=0; j<3; j++){		//  read 24 bits of channel data from 1st ADS in 8 3 byte chunks
			inByte = transferSPI( 0x00);
			channelData[i] = (channelData[i]<<8) | inByte;
		}
	}
	
	
	
	//reformat the numbers

	for( i=0; i<nchan; i++){			// convert 3 byte 2's compliment to 4 byte 2's compliment		
		if( (channelData[i] & 0x00800000) == 0x00800000 )	{
			channelData[i] = ~channelData[i];
			channelData[i] += 0x00000001;
		}else{
			//channelData[i] &= 0x00FFFFFF;
		}
		channelData[i] = channelData[i]  << 8;
	}

}

//read data
void ADS_RDATA() {				//  use in Stop Read Continuous mode when DRDY goes low
	uint8_t inByte,inByte1,inByte2,inByte3;
	int i,j;
	int nchan = 8;	//assume 8 channel.  If needed, it automatically changes to 16 automatically in a later block.
	
	stat_1 = 0;							//  clear the status registers
	

	for(i = 0; i < nchan; i++){
		channelData[i] = 0;
	}
	
	transferSPI( ADS_OPCODE_RDATA);
	
	// READ CHANNEL DATA FROM FIRST ADS IN DAISY LINE
	for(i = 0; i < 3; i++){			//  read 3 byte status register (1100+LOFF_STATP+LOFF_STATN+GPIO[7:4])
		inByte = transferSPI( 0x00);
		stat_1 = (stat_1<<8) | inByte;				
	}
	
	for(i = 0; i < 8; i++){
			inByte1 = transferSPI( 0x00);
			inByte2 = transferSPI( 0x00);
			inByte3 = transferSPI( 0x00);
		
		channelData[i] = (inByte1 << 16) | (inByte2 << 8) | inByte3;
		
	}
	
	for( i = 0; i<nchan; i++){			// convert 3 byte 2's compliment to 4 byte 2's compliment	
		//if(bitRead(channelData[i],23) == 1){	
		if( (channelData[i] & 0x00800000) == 0x00800000  ){	
			channelData[i] = ~channelData[i] + 1;
			
		}else{
			channelData[i] &= 0x00FFFFFF;
		}
		//channelData[i] = channelData[i] << 8;
	} 

}

// String-Byte converters for RREG and WREG

int32_t* getChannelData(){
	return channelData;
}
