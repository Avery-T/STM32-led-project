/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include <stdlib.h>
#include <stdio.h>
#include "ir_codes.h"
#include <math.h>
#include "arm_math.h"

#define PI 3.14159265358979323846
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
typedef struct{
	uint16_t red;
	uint16_t green;
	uint16_t blue;
}RGB_Value;

typedef struct{
	RGB_Value buffer[60];
	uint16_t head;
	uint16_t tail;
}Circular_Buffer;


void TIM2_IRQHandler(void);
void ADC1_2_IRQHandler(void);
void Timer2_Init(void);
void Timer3_Init(void);
void ADC_init(void);
void send(Circular_Buffer * rgb_buffer);
void shoot();
void USART_Init(void);
void USART_Print(char* message);
//uint32_t DWT_Delay_Init(void);

void DWT_Delay_us(uint32_t microseconds);
#define CALIBRATE(X) (uint32_t) ( (0.0008*X - 0.0276) * 1000000 )
#define ARR(WANTED_FREQ) (uint32_t)((F_CLK/WANTED_FREQ)-1)
#define MAX_LED 60
#define USE_BRIGHTNESS 1
#define F_CLK 32000000

uint16_t datasentflag = 0;

//uint8_t LED_Data[MAX_LED][4];
//uint16_t pwmData[ 24*(MAX_LED) + 50]; //+ 50 stores the reset code
//uint16_t datasentflag = 0;
volatile uint8_t ADC_flag = 0;
volatile uint32_t ADC_Value = 0;
volatile uint32_t MicroSecond_Counter = 0;


void Wait_us(uint32_t);


void GPIO_Init_Input(void);


double cutoff_frequency = 600;
double sample_rate = 40000;
#define BLOCK_SIZE 100
#define FILTER_ORDER 6
#define SAMPLING_FREQ 40000.0f  // Replace with your actual sampling frequency
#define CUTOFF_FREQ 1500.0f    // Replace with your desired cutoff frequency
#define NUM_STAGES 1  // Butterworth filter has two biquad stages for each pole

float32_t biquadCoeffs[5 * NUM_STAGES];  // Coefficients array

// Function to calculate Butterworth filter coefficients
arm_biquad_cascade_df2T_instance_f32 S;
void calculateButterworthCoeffs(void) {


    arm_biquad_cascade_df2T_init_f32(&S, FILTER_ORDER, biquadCoeffs, NULL);
}
// Function to apply Butterworth filter to input data
void applyButterworthFilter(float32_t* input, float32_t* output, uint32_t blockSize) {
    arm_biquad_cascade_df2T_f32(&S, input, output, blockSize);
}
void butterworth_lowpass( int order, uint32_t *input, double *output, uint32_t length) {
    double RC = 1.0 / (cutoff_frequency * 2 * PI);
    double dt = 1.0 / sample_rate;
    double alpha = dt / (RC + dt);
    double beta = (RC - dt) / (RC + dt);

    // Apply the difference equation for a 2nd-order lowpass Butterworth filter repeatedly for the desired order
    for (int i = 0; i < order; ++i) {
        double y = 0.0;
        for (uint32_t j = 0; j < length; ++j) {
            y = alpha * input[j] +(1-alpha) * y;
            output[j] = y;
        }
        for (size_t j = 0; j < length; ++j) {
                    input[j] = output[j];
                }
    }
}
void circular_buffer_init(Circular_Buffer * buffer)
{
	buffer->head = 0;
	buffer->tail = 0;
}

void circular_buffer_insert_val(Circular_Buffer * buffer, RGB_Value value) {
	buffer->buffer[buffer->tail] = value;
	buffer->tail = (buffer->tail + 1) % MAX_LED;

	buffer->buffer[buffer->tail] = value;
	buffer->tail = (buffer->tail + 1) % MAX_LED;
	buffer->head = (buffer->head + 1) % MAX_LED;

	  // Check if the buffer is full and update the head accordingly
	if (buffer->tail == buffer->head)
	   buffer->head = (buffer->head + 1) % MAX_LED;


}




/* USER CODE BEGIN PFP */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}
volatile uint8_t KEY_PRESSED_FLAG = 0;
uint32_t get_ir_remote_key_press();
void send_stationary_color_to_led_rgb_controller(RGB_Value rgb_buffer[]);
void set_led_color(uint16_t red, uint16_t green, uint16_t blue, double brightness);
void audio_vizulaizer();
void christmas_lights();

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  ADC_init();
  Timer2_Init();
  // Timer3_Init();
  USART_Init();

  //GPIO_Init_Input();

  //shoot();




  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  GPIOC->MODER &= ~(GPIO_MODER_MODE0); // Clear the mode bits for the pin also sets as input
  GPIOC->PUPDR |= 1; // set pupdr as no pull up pull down
  //GPIOC->MODER |= GPIO_MODER_MODE0_0;

  uint32_t key = 0;
  char inputed_freq[10] = {0};
  //USART_Print("hi");
  double brightness = 1;
  RGB_Value saved_color = {0,0,0};
  while (1)
  {
	  //KEY_PRESSED_FLAG = 0; //turn the flag off and get the key
	  key = get_ir_remote_key_press(); //returns if a key is pressed
	  USART_Print("\x1B[2J");//clear screen
	  USART_Print("\x1B[H");//move the curser to the top

	  KEY_PRESSED_FLAG = 0;
	  switch(key)
	  {
	  	case KEY_0:

	  		 USART_Print("Key0");
	  		 set_led_color(0,0,0, brightness);
	  		 break;
	  	case KEY_1:
	  		set_led_color(255,0,0,brightness);

	  		 USART_Print("Key1");
	  		 break;
	  	case KEY_2:
	  		set_led_color(0,255,0,brightness);
	  		 USART_Print("Key2");
	  		 break;
	  	case KEY_3:
	  		set_led_color(0,0,255,brightness);
	 	  	 USART_Print("Key3");
	 	  	 break;
	 	case KEY_4:
	 		christmas_lights();
	 		 USART_Print("Key4");
	 		 break;
	    case KEY_5:
	    	 USART_Print("Key5");
	         break;
	    case KEY_6:
	    	USART_Print("Key6");
	    	break;
	    case KEY_7:
	    	USART_Print("Key7");
	    	break;
	    case KEY_8:
	    	USART_Print("Key8");
	    	break;
	    case KEY_9:
	    	USART_Print("Key9");
	    	break;
	    case KEY_STAR:
	    	USART_Print("Star");
	    	break;
	    case KEY_POUND:
	    	USART_Print("POUND");
	    	break;
	    case KEY_UP_ARROW:
	    	USART_Print("UP ARROW");
	    	if(brightness<1)
	    		brightness += .1;
	    	break;
	    case KEY_RIGHT_ARROW:
	    	USART_Print("RIGHT ARROW");
	    	break;
	    case KEY_DOWN_ARROW:
	    	if(brightness>0)
	    		    brightness -= .1;
	    	USART_Print("DOWN ARROW");
	    	break;
	    case KEY_LEFT_ARROW:
	    	USART_Print("LEFT ARROW");
	    	break;
	    case KEY_MIDDLE_BTN:
	    	audio_vizulaizer();
	    	USART_Print("MIDDLE BTN");
	    	break;
	    case KEY_REPEAT:
	    	USART_Print("REPEAT");
	    	break;
	    default:
	    	break;
	  }

	  HAL_Delay(400);


	  //return;


	  //GPIOC->BSRR |= GPIO_BSRR_BS_0; // reset the gpio pin
	 // while((GPIOC->IDR & GPIO_IDR_ID0) == 1);
		//  GPIOC->BSRR |= GPIO_BSRR_BS_0; // reset the gpio pin



	  //while (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0));   // wai








  }

  }



void audio_vizulaizer()
{

	calculateButterworthCoeffs();
	  Circular_Buffer rgb_buffer;
	  circular_buffer_init(&rgb_buffer);
	  RGB_Value red = {255,0,0};
	  RGB_Value blue = {0,0,255};
	  RGB_Value off = {0,0,0};


	 // fill buffer with nothing
	  for(int i =0; i<=MAX_LED; i++)
	  {
	     circular_buffer_insert_val(&rgb_buffer,off);
	  }
	  //while((GPIOC->IDR & GPIO_IDR_ID0) ==0); // wait for the key to reset state
	  HAL_Delay(700);
	  uint32_t num_samples = 10;
	  float32_t samples[BLOCK_SIZE] = {0};

	  float32_t filter_output[BLOCK_SIZE] = {0};
	  uint32_t sample_cnt = 0;
	  char message[33];
	  while((GPIOC->IDR & GPIO_IDR_ID0)==1) //while a key is not pressed
	  {
		 // if((GPIOC->IDR & GPIO_IDR_ID0 )== 0) KEY_PRESSED_FLAG =1;  //if ID0 is zero then its pressed
		if(ADC_flag)
		{

			samples[sample_cnt] =  (float32_t)ADC_Value;

			//sprintf( message,"%d ",ADC_Value);
			//SUSART_Print(message);
			sample_cnt +=1;
			if(sample_cnt >=BLOCK_SIZE)
			{
				applyButterworthFilter(samples,filter_output,BLOCK_SIZE);
				//USART_Print("Filter ");
				for(int i = 0; i<BLOCK_SIZE; i++)
				{

					sprintf( message,"%ld ",(uint32_t) filter_output[i]);
					USART_Print(message);

					if(filter_output[i] >9)
					{
					circular_buffer_insert_val(&rgb_buffer,red);
					send(&rgb_buffer);
					}
					else
						circular_buffer_insert_val(&rgb_buffer,off);

				}

			  sample_cnt = 0;
			}

		 ADC_flag = 0;

	  }
	  else
		 circular_buffer_insert_val(&rgb_buffer,off);



	  HAL_Delay(1);
	  send(&rgb_buffer);
	  circular_buffer_insert_val(&rgb_buffer,off);


	 }


   }

void christmas_lights()
{
	RGB_Value led[60];
	for(int i = 0; i<MAX_LED; i=i+2)
	{
		led[i].red =  255;
		led[i].blue = 0;
		led[i].green =0;

		led[i+1].blue = 0;
		led[i+1].red =  0;
		led[i+1].blue = 0;
		led[i+1].green =255;

		//orange
		//led[i+2].green = 165;
		//led[i+2].red =  255;
		//led[i+2].blue = 0;
		//led[i+2].green =0;
		//blue
		//led[i+3].green = 0;
		//led[i+3].red =  0;
		//led[i+3].blue = 255;
	}
		send_stationary_color_to_led_rgb_controller(led);

}

void set_led_color(uint16_t red, uint16_t green, uint16_t blue, double brightness)
{
	RGB_Value led[60];
	for(int i = 0; i<MAX_LED; i++)
	{
		led[i].red =  red * brightness;
		led[i].blue = blue * brightness;
		led[i].green = green * brightness;
	}
	send_stationary_color_to_led_rgb_controller(led);
}

uint32_t get_ir_remote_key_press()
{
	 //USART_Print("hi");
		 uint32_t data = 0;
		 uint8_t count = 0;
		  while((GPIOC->IDR & GPIO_IDR_ID0) == 1);
		 // wait for the pin to go low 9.1 ms low
		 // USART_Print("2o2 ");

		  //GPIOC->BSRR |= GPIO_BSRR_BS_0; // reset the gpio pin

		  while((GPIOC->IDR & GPIO_IDR_ID0) ==0); // wait for the pin to high
			 // USART_Print("2o2 ");
		  //this is the start of the data frame
		  while((GPIOC->IDR & GPIO_IDR_ID0) == 1); // wait for the pin to go low agian // high for 4.5ms
		  //USART_Print("2o2 ");

		  for(int i = 0; i<32; i++ )
		  {
			  count = 0;
			  while((GPIOC->IDR & GPIO_IDR_ID0) ==0); // wait for the pin to high
			  while((GPIOC->IDR & GPIO_IDR_ID0) ==1){
				  count++;

				  if(count ==2)
				  {
					  data |= 1 << (31-i);
					  //while((GPIOC->IDR & GPIO_IDR_ID0) ==1); //make sure its done
				  }
				  if(count == 1){
					  data &= ~(1<<(31-i)); //asumes its a zero
					  DWT_Delay_us(700); //if where still high after 100 ms then continue counting
				  }
			  }
		  }
		  return data;
}


void DWT_Delay_us(uint32_t microseconds) {
    // Enable the DWT cycle counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Calculate the number of cycles for the requested delay
    uint32_t cycles = microseconds * (F_CLK / 1000000);
    /* 3 NO OPERATION instructions */
    //    __ASM volatile ("NOP");
    //   __ASM volatile ("NOP");
    // __ASM volatile ("NOP");

    // Wait until the required number of cycles has passed
   while (DWT->CYCCNT < cycles) {
        // Do nothing, just wait
    }
}

//void GPIO_init
/* USER CODE BEGIN */


void send_stationary_color_to_led_rgb_controller(RGB_Value rgb_buffer[])
{

	uint8_t LED_Data[MAX_LED][4];
	uint16_t pwmData[ 24*(MAX_LED) + 50]; //+ 50 stores the reset code

	uint32_t indx=0;
    uint32_t color=0;

	for (int i= 0; i<MAX_LED; i++)
			{

				color = (rgb_buffer[i].green<<16) | (rgb_buffer[i].red<<8) | (rgb_buffer[i].blue);

				for (int i=23; i>=0; i--)
				{
					if (color&(1<<i))
					{
						pwmData[indx] = 26;  // 2/3 of 36
					}

					else pwmData[indx] = 13;  // 1/3 of 36

					indx++;
				}

			}

			for (int i=0; i<50; i++)
			{
				pwmData[indx] = 0;
				indx++;
			}

			HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
			while (!datasentflag){};
			datasentflag = 0;
}



void send( Circular_Buffer * rgb_buffer)
{
	uint8_t LED_Data[MAX_LED][4];
	uint16_t pwmData[ 24*(MAX_LED) + 50]; //+ 50 stores the reset code
	//uint16_t datasentflag = 0;

	uint32_t indx=0;
	uint32_t color;

	uint32_t buffer_index = rgb_buffer->tail -1;
	if(buffer_index<0)
	{
		buffer_index = MAX_LED -1;
	}

		for (int i= 0; i<MAX_LED; i++)
		{

			color = ((rgb_buffer->buffer[buffer_index].green<<16) | (rgb_buffer->buffer[buffer_index].red<<8) | (rgb_buffer->buffer[buffer_index].blue));
			buffer_index = (buffer_index - 1 + MAX_LED) % MAX_LED;
			for (int i=23; i>=0; i--)
			{
				if (color&(1<<i))
				{
					pwmData[indx] = 26;  // 2/3 of 36
				}

				else pwmData[indx] = 13;  // 1/3 of 36

				indx++;
			}

		}

		for (int i=0; i<50; i++)
		{
			pwmData[indx] = 0;
			indx++;
		}

		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
		while (!datasentflag){};
		datasentflag = 0;
}

//this timer calls the ADC and also checks if a key is pressed, if a key is pressed IDR goes low
//key pressed flag is used for blocking led functions
void TIM2_IRQHandler(void) {

       TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
       //KEY_PRESSED_FLAG = 0;

       ADC1->CR |= ADC_CR_ADSTART; //call the ADC in the TIMER to controll the sampeling
//        	ADC_flag =0;
}


void ADC1_2_IRQHandler()
{
	if(ADC1->ISR & ADC_ISR_EOC)
	{
		//enqueue(sample_buffer, ADC1->DR);
		ADC_Value = ADC1->DR; //ADC_Value
		ADC_flag = 1;

		//ADC_flag = 1;
		//if(prev_sample > curr_sample)

		//   GPIOC->ODR ^= GPIO_PIN_0;

	}
}

void Timer2_Init() {

    // Enable the peripheral clock for TIM2
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    // Configure the GPIO pin for PWM output (change these values for your specific pin)
    GPIOC->MODER &= ~(GPIO_MODER_MODE0); // Clear the mode bits for the pin
    GPIOC->MODER |= GPIO_MODER_MODE0_0;
    // Configure TIM2 as PWM mode
    //TIM2->PSC = 1;
    TIM2->ARR =  ARR(40000);
    TIM2->DIER |= TIM_DIER_UIE; //flag for arr overflow inturupt

     //TIM2->DIER |= TIM_DIER_CC1IE; //intrupt enable othher thing is intrupt flag
    NVIC_SetPriority(TIM2_IRQn, 0); // Set priority (adjust as needed)
    NVIC_EnableIRQ(TIM2_IRQn); // Enable the interrupt    // Set the PWM period (ARR register)
    // Set the initial PWM duty cycle (CCR1 register)
    //TIM2->PSC = 0;
    //TIM2->CCR1 = 399; // For a 50% duty cycle, assuming a 1 kHz PWM frequency
    TIM2->CR1 |= TIM_CR1_CEN;
}


void TIM3_IRQHandler(void) {
        // Clear the update interrupt flag
       TIM3->SR &= ~TIM_SR_UIF;
       MicroSecond_Counter += 1;

}
//timer 3 interrupts at 1 us
void Timer3_Init() {

    // Enable the peripheral clock for TIM2
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    // Configure the GPIO pin for PWM output (change these values for your specific pin)
    GPIOC->MODER &= ~(GPIO_MODER_MODE0); // Clear the mode bits for the pin
    GPIOC->MODER |= GPIO_MODER_MODE0_0;
    // Configure TIM2 as PWM mode
    //TIM2->PSC = 1;
    TIM3->ARR =  ARR(100000000); // 1 us

    TIM3->DIER |= TIM_DIER_UIE; //flag for arr overflow inturupt

     //TIM2->DIER |= TIM_DIER_CC1IE; //intrupt enable othher thing is intrupt flag
    NVIC_SetPriority(TIM3_IRQn, 0); // Set priority (adjust as needed)
    NVIC_EnableIRQ(TIM3_IRQn); // Enable the interrupt    // Set the PWM period (ARR register)

    //TIM3->CR1 |= TIM_CR1_CEN;
}
void USART_Print( char * message ) {
	uint8_t i;
	for(i=0; message[i] != 0; i++){				// check for terminating NULL character
		while(!(USART2->ISR & USART_ISR_TXE));	// wait for transmit buffer to be empty
		USART2->TDR = message[i];				// transmit character to USART
	}
}

void ADC_init()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

		  ADC123_COMMON->CCR = ((ADC123_COMMON->CCR & ~(ADC_CCR_CKMODE)) | ADC_CCR_CKMODE_0);
		  ADC1->CR &= ~(ADC_CR_DEEPPWD); // take the ADC out of deep power down mode
		  ADC1->CR |= (ADC_CR_ADVREGEN);  // enable to voltage regulator gaurds the voltage
		  for(uint16_t i =0; i<1000; i++)
			  for(uint16_t j = 0;  j<100; j++); //delay at least 20us
		  //calibrate, you need to digital calibrate
		  ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); //ensure adc is not enabled, single ended calibration
		  ADC1->CR |= ADC_CR_ADCAL;       // start calinration
		  while(ADC1->CR & ADC_CR_ADCAL); // waiat for calibration

		  //configure single ended mode before enabling ADC
		  ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5); // PA0 is ADC1_IN5, single endede mode

		  //enable ADC

		  ADC1->ISR |= (ADC_ISR_ADRDY);
		  ADC1->CR |= ADC_CR_ADEN;
		  while(!(ADC1->ISR & ADC_ISR_ADRDY));
		  ADC1->ISR |= (ADC_ISR_ADRDY);


		  //configure ADC
		  ADC1->SQR1 = (ADC1->SQR1 & ~(ADC_SQR1_SQ1_Msk | ADC_SQR1_L_Msk)) |(5 << ADC_SQR1_SQ1_Pos);
		  //ADC1->CFGR |= ADC_CFGR_CONT;
		  ADC1->IER |= (ADC_IER_EOC);
		  ADC1->ISR &= ~(ADC_ISR_EOC);

		  //ADC1->SMPR1 |= 7<12; // 640.5 ADC clock cycles

		  NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));


		  __enable_irq();

		  //configure GPIO pin PA0
		  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
		  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL0)) | (7 << GPIO_AFRL_AFSEL0_Pos);
		  GPIOA->MODER |= (GPIO_MODER_MODE0); // analog mode for Pa0
		  GPIOA->ASCR |= GPIO_ASCR_ASC0; //set Pa0 to analog

		 //ADC1->CR |= ADC_CR_ADSTART; // start conversion
}

void USART_Init()
{

	  // configure GPIO pins for USART2 (PA2, PA3) follow order of configuring registers
	  // AFR, OTYPER, PUPDR, OSPEEDR, MODDER otherwise a glitch is created on the output pin
	  //RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);		// mask AF selection
	  GPIOA->AFR[0] |= ((7 << GPIO_AFRL_AFSEL2_Pos ) |				// select USART2 (AF7)
	  		  	  	   (7 << GPIO_AFRL_AFSEL3_Pos)); 		  	  	// for PA2 and PA3
	  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);		// push-pull output for PA2, PA3
	  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);		// no pull ups on PA2, PA3
	  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED2);	// low speed
	  GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);		// enable alternate function
	  GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);    // for PA2 and PA3

	  // Configure USART2 connected to the debugger virtual COM port
	  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;			// enable USART by turning on system clock
	  USART2->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);	//set data to 8 bits
	  USART2->BRR = 32000000 / 115200;                     //over sampling BRR formula
	  USART2->CR1 |= USART_CR1_UE;						// enable USART
	  USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);		// enable transmit and receive for USART

	  // enable interrupts for USART2 receive
	  USART2->CR1 |= USART_CR1_RXNEIE;					// enable RXNE interrupt on USART2
	  USART2->ISR &= ~(USART_ISR_RXNE);					// clear interrupt flagwhile (message[i] != 0)

	  NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));		// enable USART2 ISR
	  __enable_irq();
}
/* USER CODE BEGIN */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 39;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
