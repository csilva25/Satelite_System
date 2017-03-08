
#include "stm32F30x.h"
#include "STM32f3_discovery.h"
#include "stm32f30x_gpio.h"
#include  "stm32f30x_i2c.h"
#include  "stm32f30x_it.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include "main.h"
#include "math.h"
#include  "stdio.h"
#include  "string.h"

#define ABS(x)         (x < 0) ? (-x) : x

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f  

#define		SYSTICK_MASK	0x8000
#define		TIM3_MASK		0x0200

#define MOTOR_A_PWM_PIN GPIO_Pin_12             //D12
#define MOTOR_B_PWM_PIN GPIO_Pin_13             //D13



__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0, calibration_value = 0;
__IO uint16_t  ADC2ConvertedValue = 0, ADC2ConvertedVoltage = 0, calibration_value2 = 0;

__IO uint32_t TimingDelay = 0;
ADC_InitTypeDef       ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
RCC_ClocksTypeDef 			RCC_Clocks;
GPIO_InitTypeDef        GPIO_InitStructure;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_InitTypeDef RTC_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;
RTC_AlarmTypeDef RTC_AlarmStructure;
NVIC_InitTypeDef NVIC_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
__IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;
__IO uint8_t showtime[50] = {0};
__IO uint8_t showdate[50] = {0};

void Demo_GyroConfig(void);
void Demo_GyroReadAngRate (float* pfData);
void Demo_CompassConfig(void);
void Demo_CompassReadMag (float* pfData);
void Demo_CompassReadAcc(float* pfData);
void Read_Compass(float *, float *, float *);


void 	InitPwmGpio(void);
int    	InitPwmSignal(int);
void    RTC_Setup(void);
void    Init_GPIOE(void);
void    Init_Keypad(void);
void    Init_LCD(void);
void  LCD_cursor(void);
void  LCD_setpos(int row, int col);

void  LCD_write(int,int, char); // hor, vert, charactor
void  LCD_clear(void);
void  LCD_contrast(int);                // Contrast level = 1..50
void  LCD_backlight(int);               // Backlight level = 1..8
void Delay(uint32_t nTime);

void  ADC(void);
int  ADC1_Read(void);
int  ADC2_Read(void);

void  LED(void);
void MoveLED(float pre_val,float pre_val2);
void MoveServoArm(float position,float position2);
 int x,y,xa=4,ya=4,adc_yvalue,adc_xvalue,tick_count = 0;
int pre_val=0,pre_val2=0;
volatile int     ButtonPressed = 0;
volatile int arrayx[8]={570,1140,1710,2280,2900,3400,4000,6096};
volatile int arrayy[10]={-48,-36,-24,-12,12,24,36,48,60};
char        message[16];
float       Heading, Xangle, Yangle;
int pulse_width1,pulse_width2;

int main(void) {
	int i;
	Init_GPIOE();
	Init_LCD();
	InitPwmGpio();
	InitPwmSignal(20);
	Demo_CompassConfig();
	ADC();
	
	while(1){
		
		
		
while(!ButtonPressed){
	Delay(10);
		
		LED();
	Delay(10);
		MoveServoArm(0,0);

			}

			
		//while loop that happenes first before the user button is pressed 	
while(ButtonPressed){
	
	
		Read_Compass(&Heading, &Xangle, &Yangle);
	
			sprintf(message, "X=%4.2f Y=%4.2f", Xangle,Yangle);
		for (i=0; i < strlen(message); i++)
		  LCD_write(0, i, message[i]);				// Display number on 1st line of display  
		sprintf(message, "x=%d ", xa);
		for (i=0; i < strlen(message); i++)
              LCD_write(1, i, message[i]);
	
			MoveLED(Xangle,Yangle);
		Delay(10);
		MoveServoArm(Xangle,Yangle);

}
}
	}

void MoveLED(float Xangle,float Yangle) //function that moves led by referencing an array 
{
	int col,row;
	x=4,y=4; //setting led in the middle 
	adc_xvalue=(int)Xangle +0.5; //casting the float into an int
	adc_yvalue=(int)Yangle +0.5;
	

	
	
//------------- the y-axis code -------------	
if (pre_val != adc_yvalue && ya != 7  && adc_yvalue > 0
	
	&& (adc_yvalue > arrayy[ ya ] )){
		
				pre_val = adc_yvalue;
		
						ya++;}
 if (ya!=0 && adc_yvalue < arrayy[ya]  ){
						ya--;}
						
	
//------------- the x-axis code -------------	
if (pre_val2 != adc_xvalue && xa != 7  && adc_xvalue > 0
	
	&& (adc_xvalue > arrayy[ xa ] )){

				pre_val2 = adc_yvalue;
		
						xa++;}	

						
 if (xa!=0 && adc_xvalue < arrayy[xa]    ){
		xa--;
						}
												

		row =(int)pow(2,xa); //uses power function to the power of which x and y values 
		col =(int)pow(2,ya);
		GPIOD->ODR = row ;
		GPIOC->ODR = ~col;// for out led we needed to xor this
}	
	
void MoveServoArm(float pulse_width1, float pulse_width2) // passing by value
	{
		if (ButtonPressed){
		pulse_width1 = 2900+(pulse_width1 * 22); // Min value (0 degrees) + (x-angle value * scaling factor)
		pulse_width2 = 2900+(pulse_width2 * 22); // Min value (0 degrees) + (x-angle value * scaling factor)
		}
	
	
		if (!ButtonPressed){
	pulse_width1 = 1200 + adc_xvalue;/*potentiomitor (0-7)*/
	pulse_width2 = 1200 + adc_yvalue;/*potentiomitor (0-7)*/
		}
	
			Delay(20); //delay to make smoother movement 
			TIM_SetCompare1(TIM4, pulse_width1); // uses comparator for the timer 4 and passes in the new pulse width
			TIM_SetCompare2(TIM4, pulse_width2);
}//once out of scope, position is disregarded
	



int  ADC1_Read(void){ ///this reads the adc values waits until flag is down and sets the value to a variable 
	int value = 0;
		  ADC_Cmd(ADC1, ENABLE);
	
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	
	ADC_StartConversion(ADC1); 
	value =ADC_GetConversionValue(ADC1);  
	ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;	
	return value;
}




int  ADC2_Read(void){///this reads the adc values waits until flag is down and sets the value to a variable 
	int value = 0;
		  ADC_Cmd(ADC2, ENABLE);
	
  while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
	
	ADC_StartConversion(ADC2); 
	value =ADC_GetConversionValue(ADC2);  
	ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;	
	return value;
}
		

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
void  LED(void){  
int col,row,i;
	
	adc_yvalue=ADC1_Read();
	adc_xvalue=ADC2_Read();
	
//------------- the y-axis code -------------	
if (pre_val != adc_yvalue && y != 7  && adc_yvalue > 0
	
	&& (adc_yvalue > arrayx[ y ] && adc_yvalue < arrayx[ y+1 ])){
		
				pre_val = adc_yvalue;
		
						y++;}
 if (y!=0 && adc_yvalue < arrayx[y-1]  ){
						y--;}
						
	
//------------- the x-axis code -------------	
if (pre_val2 != adc_xvalue && x != 7  && adc_xvalue > 0
	
	&& (adc_xvalue > arrayx[ x ] && adc_xvalue < arrayx[ x+1 ])){

				pre_val2 = adc_yvalue;
		
						x++;}	

						
 if (x!=0 && adc_xvalue < arrayx[x-1]  ){
		x--;
						}
												


		row =(int)pow(2,x);
		col =(int)pow(2,y);
		GPIOD->ODR = row ;
		GPIOC->ODR = ~col;
						
sprintf(message,"ADC2=%d V=%d",adc_xvalue,ADC1ConvertedVoltage);
	for (i=0; i < strlen(message); i++)
						LCD_write(0, i, message[i]);
	sprintf(message,"ADC1=%d",adc_yvalue);
	for (i=0; i < strlen(message); i++)
						LCD_write(1, i, message[i]);

				
}



void  ADC(void){
  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  
  /* Enable ADC1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
      
  /* Setup SysTick Timer for 1 µsec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000000))
  { 
    /* Capture error */ 
    while (1)
    {}
  }
  
  /* ADC Channel configuration */
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure ADC Channel7 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  ADC_StructInit(&ADC_InitStructure);

  /* Calibration procedure */  
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  
  /* Insert delay equal to 10 µs */
  Delay(10);
	
	
	  while(ADC_GetCalibrationStatus(ADC2) != RESET );
  calibration_value = ADC_GetCalibrationValue(ADC2);
     
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC2, &ADC_CommonInitStructure);

	
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 2;
  ADC_Init(ADC2, &ADC_InitStructure);

  
  /* ADC1 regular channel7 configuration */ 
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);
	
  /* ADC1 regular channel7 configuration */ 

  /* Enable ADC1 */
  /* Enable ADC1 */




//-----------------Second Potenti`-------------
//
//
//
  if (SysTick_Config(SystemCoreClock / 1000000))
  { 
    /* Capture error */ 
    while (1)
    {}
  }
  
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* Configure ADC Channel7 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  ADC_StructInit(&ADC_InitStructure);

  /* Calibration procedure */  
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	

  
  /* Insert delay equal to 10 µs */
  Delay(10);
	  while(ADC_GetCalibrationStatus(ADC1) != RESET );
		calibration_value2 = ADC_GetCalibrationValue(ADC1);
     
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          

	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 2;

	ADC_Init(ADC1, &ADC_InitStructure);
  
    /* ADC1 regular channel7 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_7Cycles5);

	 ADC_Cmd(ADC1, ENABLE);
  /* Start ADC1 Software Conversion */ 
	 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  
 ADC_StartConversion(ADC1);  
  /* Test EOC flag */
    /* Get ADC1 converted data */
			
			ADC2ConvertedValue =ADC_GetConversionValue(ADC1);  
    /* Compute the voltage */
		
		 ADC2ConvertedVoltage = (ADC2ConvertedValue *3300)/0xFFF;	
		/* Configure ADC Channel7 as analog input */
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

void Init_GPIOE() {
  
		EXTI_InitTypeDef   EXTI_InitStructure;
		GPIO_InitTypeDef   GPIO_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;
			 
			
		/* SysTick end of count event each 1ms */
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);


		/* GPIOE Periph clock enable */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12
			| GPIO_Pin_11| GPIO_Pin_10| GPIO_Pin_9| GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		
		
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 
		| GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;              
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		


			/* Enable GPIOA clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		/* Configure PA0 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable SYSCFG clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		/* Connect EXTI0 Line to PA0 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
		

		/* Configure EXTI0 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI0 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

}




void Init_LCD(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		I2C_InitTypeDef  I2C_InitStructure;

		RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);  

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);  

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		I2C_DeInit(I2C2);
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  
		I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
		I2C_InitStructure.I2C_DigitalFilter = 0x00;
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_Timing = 0xC062121F; 

		I2C_Init(I2C2, &I2C_InitStructure);  
		I2C_Cmd(I2C2, ENABLE);
		LCD_clear();
		
}



void InitPwmGpio()
{

	
	
   
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = MOTOR_A_PWM_PIN | MOTOR_B_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; /* Use the alternative pin functions */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /* GPIO speed - has nothing to do with the timer timing */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* Push-pull */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; /* Setup pull-down resistors */
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Connect the timer output to the LED pins

   GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2); /* TIM4_CH1 -> MOTOR_A_PWM */
	 GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_2); /* TIM4_CH1 -> MOTOR_A_PWM */
}


int InitPwmSignal(int pwm_freq)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    double ms_pulses;

    /* Calculates the timing. This is common for all channels */
    int clk = 72e6; 		// 72MHz -> system core clock. Default on the stm32f3 discovery
    //int clk = 36e6; 		// (APB1 max) 36MHz. Default on the stm32f3 discovery

    int tim_freq = 2e6; 	// in Hz (2MHz) Base frequency of the pwm timer

    int prescaler = ( (clk / tim_freq) - 1 );

    // Calculate the period for a given pwm frequency
    int pwm_period = tim_freq / pwm_freq;       // 2MHz / 50Hz = 40000


    // Calculate a number of pulses per millisecond.
    ms_pulses = (float)pwm_period / ( 1000.0 / pwm_freq ); // for 50Hz we get: 40000 / (1/50 * 1000)


    //  Enable the TIM4 peripheral
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE );

    // Setup the timing and configure the TIM4 timer
    TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_Period = pwm_period - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


    // Initialize the OC for PWM
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (int)(ms_pulses*10); 	// preset pulse width 1 ms
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	// Pulse polarity

    // Setup channels
    // Channel 1 (PD12)
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // Channel 2  (PD13)
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // Start the timer
    TIM_Cmd(TIM4 , ENABLE);

    return pwm_period;
}


void Read_Compass (float *Hptr, float *Xptr, float *Yptr) {
  int   i;
 __IO float HeadingValue = 0.0f;
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f};

float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
float fTiltedX,fTiltedY = 0.0f;



  
 /* Read Compass data */
      Demo_CompassReadMag(MagBuffer);
      Demo_CompassReadAcc(AccBuffer);
      
      for(i=0;i<3;i++)
        AccBuffer[i] /= 100.0f;
      
      fNormAcc = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));
      
      fSinRoll = -AccBuffer[1]/fNormAcc;
			fCosRoll =  sqrt(1.0f-(fSinRoll * fSinRoll));
      fSinPitch = AccBuffer[0]/fNormAcc;
      fCosPitch = sqrt(1.0f-(fSinPitch * fSinPitch));
 
      RollAng = ((atan2(fSinRoll, fCosRoll))*180/PI);
      PitchAng = ((atan2(fSinPitch, fCosPitch))*180/PI);
            
      fTiltedX = MagBuffer[0]*fCosPitch+MagBuffer[2]*fSinPitch;
      fTiltedY = MagBuffer[0]*fSinRoll*fSinPitch+MagBuffer[1]*fCosRoll-MagBuffer[1]*fSinRoll*fCosPitch;
      
      HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;
 
      if (HeadingValue < 0)
      {
        HeadingValue = HeadingValue + 360;    
      }
      
      
      *Hptr = HeadingValue;
      *Xptr = PitchAng;
      *Yptr = RollAng;
}

void Demo_GyroConfig(void)
{
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
  
  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
  L3GD20_Init(&L3GD20_InitStructure);
   
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
void Demo_GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)RawData[i]/sensitivity;
  }
}

/**
  * @brief  Configure the Mems to compass application.
  * @param  None
  * @retval None
  */
void Demo_CompassConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
  
  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);
  
   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);
  
  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void Demo_CompassReadAcc(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
  
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);
   
  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  }

}

/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void Demo_CompassReadMag (float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);
  
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }
  
  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
  
}





/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  return 0;
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}


//
//      Set LCD Cursor Position - row = 0,1  col = 0..15
//
void  LCD_setpos(int row, int col) {
        
        // Move to sepcified row, col
  
        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);
          
        I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
      
        I2C_SendData(I2C2, 0xFE);
               
        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
      
        I2C_SendData(I2C2, 0x45);
        
  
        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);        
        if (!row)               // if row == 0
            I2C_SendData(I2C2, col);
         else                  // else row asumed to be 1
            I2C_SendData(I2C2, (0x40 + col));       
         
        
}         

//
//      Turn on LCD Cursor - Makes cursor visible
//
void  LCD_cursor(void) {
  
        //while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);
          
        I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
      
        I2C_SendData(I2C2, 0xFE);
               
        while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
      
        I2C_SendData(I2C2, 0x47);  	// underline   (Blinking = 0x4B)
      
        Delay(20);
}         


void  LCD_write(int row, int col, char data) {
          
		// Move to sepcified row, col
			
		I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
					 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x45);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);        
		if (!row)               // if row == 0
				I2C_SendData(I2C2, col);
		 else                  // else row asumed to be 1
				I2C_SendData(I2C2, (0x40 + col));       
		 
		I2C_TransferHandling(I2C2, 0x50 , 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
			
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);                        
		I2C_SendData(I2C2, data);

}         


//
//      Set LCD Contrast - Level should be 1..50 (Seems to work best if > 35)
//

void  LCD_contrast(int level) {
     
		I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
					 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x52);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, level); 

		Delay(20);
}         

//
//      Set LCD Backlight - Level should be 1..8 (Seems to work best if > 1)
//

void  LCD_backlight(int level) {
  

		I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
				 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x53);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, level);
			
		Delay(20);
}         


void  LCD_clear() {
       
		I2C_TransferHandling(I2C2, 0x50 , 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
					 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x51);

		Delay(20);
}         

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
  
}


void SysTick_Handler(void)
{
     TimingDelay_Decrement();
     
     tick_count++;
}


void EXTI0_IRQHandler(void)
{

	if (EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) == SET)
  {
		
		
    if (!ButtonPressed) ButtonPressed=1;
		else	ButtonPressed=0;
		
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  }    

}

