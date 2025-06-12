#include "N76E003.h"

// i2c variable start //
#define I2C_CLOCK            13
#define EEPROM_SLA           0xA4     //0x15 //0x20  //0xA4  //   // 

void Init_I2C(void);

unsigned char temp _at_ 0x08;
unsigned char idata itemp _at_ 0x80;
unsigned char xdata xtemp _at_ 0x80;

uint16_t findMax(uint16_t arr[], uint8_t size);
double map(long x, float in_min, float in_max, float out_min, float out_max);

#define CTSampleSize  10
#define avgSamples 		50
uint16_t CT[CTSampleSize]={0,};

signed int adcval = 0;
uint32_t adcAvg = 0;
uint16_t finalADCAVG = 0;
uint8_t sampleCounter = 0;
uint16_t x = 0x24A6;
/*==========================================================================*/
void main (void) 
{
	uint8_t a=0;
		InitialUART1_Timer3(115200);
		TI_1 = 1;
		Enable_ADC_AIN1;						// Enable AIN1 P3.0 as ADC input, Find in "Function_define.h" - "ADC INIT"
	
	/* Initial I2C function */
    Init_I2C();                                 //initial I2C circuit

		while(1)
		{
			adcval = 0;
			for(a=0; a<CTSampleSize; a++){
				clr_ADCF;
				set_ADCS;									// ADC start trig signal
				while(ADCF == 0);
				adcval = ADCRH;
				adcval = adcval << 4;
				adcval |= (ADCRL & 0x0f);
				CT[a] = adcval;
				Timer0_Delay1ms(2);
			}
			adcAvg = (adcAvg + findMax(CT, CTSampleSize));
			sampleCounter++;
			if(sampleCounter >= avgSamples){
				Timer0_Delay1ms(20);
				sampleCounter = 0;
				finalADCAVG = (uint16_t)adcAvg/avgSamples;
				adcAvg = 0;
			}	
	}
}

uint16_t findMax(uint16_t arr[], uint8_t size){
	int i=0;
	uint16_t max=0;
	for (i=0; i<size; i++){
		if(arr[i] > max){
			max = arr[i];
		}
	}
	return max;
}

void Init_I2C(void)
{
    P13_Quasi_Mode;                         //set SCL (P13) is Quasi mode
    P14_Quasi_Mode;                         //set SDA (P14) is Quasi mode
    
    SDA = 1;                                //set SDA and SCL pins high
    SCL = 1;
    
    set_P0SR_6;                             //set SCL (P06) is  Schmitt triggered input select.
    
    set_EI2C;                               //enable I2C interrupt by setting IE1 bit 0
    set_EA;

    I2ADDR = EEPROM_SLA;                    //define own slave address
    set_I2CEN;                              //enable I2C circuit
    set_AA;
}

void I2C_ISR(void) interrupt 6
{
	//printf("\n i2c interrupt happend %x", I2STAT);
    switch (I2STAT)
    {
        case 0x00:
            STO = 1;
            break;

        case 0x60:
            AA = 1;
            //P3 = 0x60;
						//printf("\ncase 0x60");
						//data_received[data_num] = I2DAT;
						//printf("\ndata receive inb 0x60 : %x", data_received[data_num]);
            break;
				
        case 0x68:
						P02 = 0;
            while(1);
            break;

        case 0x80:
            //P3 = 0x80;
						//printf("\ndata received\n");
						//I2C_start();
            //I2C_write_DATA(0x25);
            //I2C_stop();
				
            //data_received[data_num] = I2DAT;
						//printf("\ndata receive in 0x80 : %x", data_received[data_num]);
				/*
            data_num++;

            if (data_num == 34)
                AA = 0;
            else
                AA = 1;
				*/
            break;

        case 0x88:
            //P3 = 0x88;
            //data_received[data_num] = I2DAT;
            //data_num = 0;
            AA = 1;
            break;

        case 0xA0:
            //P3 = 0xA0;
            AA = 1;
            break;

        case 0xA8:
						I2DAT = finalADCAVG/100; // ((finalADCAVG & 0xff00)>>8);
						clr_SI;
						while (!SI);                            //Check SI set or not
						I2DAT = finalADCAVG%100; // (finalADCAVG & 0x00ff);
            // data_num++;
            AA = 1;
            break;
        
        case 0xB8:
            //P3 = 0xB8;
            //I2DAT = data_received[data_num];
            //data_num++;
            AA = 1;
            break;

        case 0xC0:
            AA = 1;
            break; 

        case 0xC8:
            //P3 = 0xC8;
            AA = 1;
            break;        
    }

    SI = 0;
//    while(STO);
}