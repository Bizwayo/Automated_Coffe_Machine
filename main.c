/*STUDENT NAME:								BIZWAYO KASARO
	STUDENT NUMBER:							7801809
	ASSIGNMENT: 								4
	ASSIGNMENT TITLE:						Scheduler
													
*/

/* 
Remarks:

I used some features provided in Assignment 1 sample solution: button_input, allOn, allOff

System clock is based on HSE configuration

PendSV, SysTcik and SVC handlers "define" have been added to port.c
#define  xPortPendSVHandler      PendSV_Handler 
#define  xPortSysTickHandler     SysTick_Handler
#define  vPortSVCHandler         SVC_Handler
*/



#include "main.h"
#include <semphr.h>

/* Global Variables */
GPIO_InitTypeDef GPIO_InitStruct; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.
TIM_HandleTypeDef TIM_InitStruct;  // Initiate the structure that contains the configuration information for the timers

//servo timer
GPIO_InitTypeDef GPIO_InitStruct_PWM; // Initiate the structure that contains the configuration information for the specified GPIO peripheral.
TIM_HandleTypeDef TIM3_InitStruct;
TIM_OC_InitTypeDef TIM3_OCInitStructure;

//Handlers for Audio
I2C_HandleTypeDef hi2c1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;


/* The task functions prototype*/
//INTERRUPTS: BUTTON, SERVO, AUDIO
void vTaskButton( void *pvParameters );
void vServo( void *pvParameters );
void vTaskBeep( void *pvParameters ) ;

//Program tasks
void vSelectSchedule( void *pvParameters );
void vFPS(void *pvParameters);
void vExperiment(void *pvParameters);


void vEspresso(void *pvParameters);
void vLatte(void *pvParameters);
void vCappuccino(void *pvParameters);
void vMocha(void *pvParameters);

//Hardware handlers
TaskHandle_t xServo;
TaskHandle_t xButtonHandler;
TaskHandle_t xSoundHandler;

//Scheduler handlers
TaskHandle_t xSelectSchedule;
TaskHandle_t xFPS;
TaskHandle_t xExperiment;

//Coffee handlers
TaskHandle_t xEspresso;
TaskHandle_t xLatte;
TaskHandle_t xCappuccino;
TaskHandle_t xMocha;


/* Function prototypes */
void SystemClock_Config(void);
void InitGPIO(void);
void InitLED(void);
void InitButton(void);
void InitTimers(void);

Button create;

Coffee espresso;
Coffee latte;
Coffee cappuccino;
Coffee mocha;

Coffee list[4]; // sorted list of tasks
tone t; // completion tone

xSemaphoreHandle xBinarySemaphore;

//======================================================================================================

/*
Use SysTick_Handler to wake xPortSysTickHandler when the FreeRTOS's 
scheduler is started. SysTick_Handler (HAL_IncTick) is needed by 
HAL driver such as DAC that depends on system tick but it's "conflict" 
with FreeRTOS xPortSysTickHandler because the SysTimer is used by FreeRTOS. 
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
	
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
			xPortSysTickHandler();
  }
	
}



/** ISR Function **/
void EXTI0_IRQHandler(void) 
{
	// Reset existing interrupt at GPIO_PIN_0
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0); //alternative: HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0)
	
	BaseType_t xYieldRequiredCheck = xTaskResumeFromISR(xButtonHandler);
	portYIELD_FROM_ISR(xYieldRequiredCheck); // Yield to avoid delayed until the next time the scheduler
}


/** This function handles DMA1 stream5 global interrupt. **/

void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
}


void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	
  // Enable Power Control clock
  __PWR_CLK_ENABLE();

  // The voltage scaling allows optimizing the power consumption when the
  // device is clocked below the maximum system frequency, to update the
  // voltage scaling value regarding system frequency refer to product
  // datasheet.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE Oscillator and activate PLL with HSE as source
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  // Based on the STM32F407 HSE clock configuration
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  // clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
	
	// Select peripheral clock for I2S
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 100;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

void InitServo(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock to GPIO-B for PB6
    // Set GPIOB Pins Parameters, PA6 
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;  //PB6: TIM4_CH1 and PB7: TIM4_CH2
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //Alternate Function Push Pull Mode 
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No Pull-up or Pull-down activation
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // Assign those pins alternate function in TIM4 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Init GPIOB
}

void InitTimer3(void)
{
		//__TIM1_CLK_ENABLE();
		__HAL_RCC_TIM3_CLK_ENABLE(); // Enable clock to TIM3 from APB2 bus (48Mhz max)xPLL_P = 84MHz
		 
	// TIM4 is configure to 50hz: 50 times in 1s or 1000000us
    TIM3_InitStruct.Instance = TIM3;
		TIM3_InitStruct.Init.Period = 20000-1;
    TIM3_InitStruct.Init.Prescaler   = 84-1;
		TIM3_InitStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
		TIM3_InitStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TIM3_InitStruct.Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(&TIM3_InitStruct); // Init TIM3
		
    HAL_TIM_Base_Start(&TIM3_InitStruct); // Start TIM3
}

void InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  //__HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level LEDs and DAC output */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


void InitButton(void)
{
	  __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock to GPIO-A for button
    // Set GPIOA Pin Parameters, pin 0 for button
    GPIO_InitStruct.Pin     = GPIO_PIN_0;
	  // Specifies the trigger signal active edge for the EXTI lines. (e.g. Rising = button is pressed)
    GPIO_InitStruct.Mode    = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Init GPIOA
		HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn); // Enable GPIO_PIN_0 interrupt at IRQ-Level
}	


 //I2C1 init function */
void InitI2C1(void)
{
		hi2c1.Instance = I2C1;
		hi2c1.Init.ClockSpeed = 100000; // clock frequency <= 400kHz
		hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; //fast mode Tlow/Thigh = 2
		hi2c1.Init.OwnAddress1 = 0; 
		hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c1.Init.OwnAddress2 = 0; 
		hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		HAL_I2C_Init(&hi2c1); //initialize the I2C peripheral 
}

/* I2S3 init function */
void InitI2S3(void)
{
		hi2s3.Instance = SPI3;
		hi2s3.Init.Mode = I2S_MODE_MASTER_TX; //transmit in master mode
		hi2s3.Init.Standard = I2S_STANDARD_PHILIPS; //data protocol
		hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B; //bits per sample
		hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE; //master clock signal
		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K; //audio frequency
		hi2s3.Init.CPOL = I2S_CPOL_LOW; //clock polarity
		hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
		hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
		HAL_I2S_Init(&hi2s3); //set the setting 
}

/** 
  * Enable DMA controller clock
  */

void InitDMA(void) 
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}


//Control the Servo using timer 3 instead
void SetupPWM_TIM3()
{
	HAL_TIM_PWM_Init(&TIM3_InitStruct);
	
	TIM3_OCInitStructure.OCMode = TIM_OCMODE_PWM1; //Set output capture as PWM mode
  TIM3_OCInitStructure.Pulse = 0; // Initial duty cycle at 0%
  TIM3_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH; // HIGH output compare active
  TIM3_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE; // output compare disable
	HAL_TIM_PWM_ConfigChannel(&TIM3_InitStruct, &TIM3_OCInitStructure, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM3_InitStruct, TIM_CHANNEL_1); // Start PWM at channel 1
	HAL_TIM_PWM_ConfigChannel(&TIM3_InitStruct, &TIM3_OCInitStructure, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM3_InitStruct, TIM_CHANNEL_2); // Start PWM at channel 2
}


/*-----------------------------------------------------------*/


//=============================================================================================================================

void allON(){ //All LEDs ON
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); //red
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); //blue
}

void allOff(){ //ALL LEDs OFF
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}


void createCoffee(){ //Fills coffee with details
	espresso.deadline = 5;  latte.deadline = 7;   cappuccino.deadline = 7;    mocha.deadline = 10;
	espresso.duration = 2;  latte.duration = 3;   cappuccino.duration = 3;    mocha.duration = 4;
	espresso.period = 10;   latte.period = 20;    cappuccino.period = 20;     mocha.period = 30;
	espresso.priority = 3;  latte.priority = 1;   cappuccino.priority = 2;    mocha.priority = 2;
	espresso.count = 0;     latte.count = 0;			cappuccino.count = 0;				mocha.count = 0;
	
	espresso.handler = xEspresso;
	latte.handler = xLatte;
	cappuccino.handler = xCappuccino;
	mocha.handler = xMocha;
}

//================================================================================================================================

int main( void )
{
  
	/* essential Board initializations
		- System Clock
		- GPIO
		- Button
		- Servo
	  - Timer3
	  - DMA
	*/
	SystemInit();
	HAL_Init();
  SystemClock_Config();
	InitGPIO();
	InitButton();
	InitServo();
	InitTimer3();
	SetupPWM_TIM3();
	InitDMA();
	InitI2C1();
	InitI2S3();
	
	//Load coffee data
	createCoffee();
	
	//xBinarySemaphore = xSemaphoreCreateBinary();
	//xSemaphoreTake(xBinarySemaphore,0);
	
	printf("FreeRTOS running on STM32F407 Discovery Board\n");

	
		xTaskCreate( vTaskButton, "vTaskButton", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, &xButtonHandler );
		xTaskCreate( vServo,"Servo",STACK_SIZE_MIN,NULL,tskIDLE_PRIORITY,&xServo );
		xTaskCreate( vTaskBeep, "vTaskBeep", STACK_SIZE_MIN*2, NULL, tskIDLE_PRIORITY, &xSoundHandler );
	
		xTaskCreate( vSelectSchedule,"Selecting Schedule",240,NULL,tskIDLE_PRIORITY,&xSelectSchedule );	
		xTaskCreate( vFPS,"FPS Scheduler",STACK_SIZE_MIN*2,NULL,tskIDLE_PRIORITY,&xFPS);
		xTaskCreate( vExperiment,"Experimental Scheduler",STACK_SIZE_MIN*2,NULL,tskIDLE_PRIORITY,&xExperiment);
		xTaskCreate( vEspresso,"Espresso",STACK_SIZE_MIN,NULL,tskIDLE_PRIORITY,&xEspresso );
		xTaskCreate( vLatte,"Latte",STACK_SIZE_MIN,NULL,tskIDLE_PRIORITY,&xLatte );
		xTaskCreate( vCappuccino,"Cappuccino",STACK_SIZE_MIN,NULL,tskIDLE_PRIORITY,&xCappuccino );
		xTaskCreate( vMocha,"Mocha",STACK_SIZE_MIN,NULL,tskIDLE_PRIORITY,&xMocha );
	
	
	
	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();
	
	for( ;; );
}
/*-----------------------------------------------------------------------------------------------------------------*/

void vTaskBeep( void *pvParameters ) // Audio Task 
{
	const char *pcTaskName = "vTaskBeep is running\n";
	for(;;)
	{
		vTaskSuspend(NULL); //Suspend itself until resume 
		//if(xSemaphoreTake(xBinarySemaphore,portMAX_DELAY)){
			printf("%s\n",pcTaskName);

			CS43_Init(hi2c1, MODE_I2S);
			CS43_SetVolume(30); //0 - 100
			CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
			CS43_Start();
			//Create Sine wave
			t.sample_dt = 880.0f/F_SAMPLE;
			t.sample_N =  F_SAMPLE/880.0f;
			for(uint16_t i=0; i<t.sample_N; i++)
			{
				t.mySinVal = AMP*sinf(i*2*PI*t.sample_dt); 
				t.dataI2S[i*2] = t.mySinVal;    //Right data (even: 0,2,4,6...)
				t.dataI2S[i*2 + 1] = t.mySinVal; //Left data  (odd: 1,3,5,7...)
			}
    
			// Output the sample through I2S from DMA
			HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)t.dataI2S, t.sample_N*2);
			vTaskDelay(1000);
			HAL_I2S_DMAStop(&hi2s3);
		
			// Reset the DAC output pin 
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_RESET);
		//}
	}
		
}

int selected = 0 ;

void vServo(void *pvParameters){
	const char *pcTaskName = "Servo is running";
	printf("%s\n",pcTaskName);
	
	for(;;){
		if(selected == 2){
			vTaskSuspend(NULL);
		TIM3->CCR1 = 500; //espress
		vTaskSuspend(NULL);
		TIM3->CCR1 = 1200; //cappa
		vTaskSuspend(NULL);
		TIM3->CCR1 = 800; //mocha
		vTaskSuspend(NULL);
		TIM3->CCR1 = 1800; //latte
		}
		else{
		vTaskSuspend(NULL);
		TIM3->CCR1 = 500; //espress
		vTaskSuspend(NULL);
		TIM3->CCR1 = 800; //mocha
		vTaskSuspend(NULL);
		TIM3->CCR1 = 1200; //cappa
		vTaskSuspend(NULL);
		TIM3->CCR1 = 1800; //latte
		}
	}
}

void vTaskButton( void *pvParameters ) // Button Task (interrupt)
{
	const char *pcTaskName = "vTaskButton is running\n";
	printf("%s\n",pcTaskName);
	TickType_t start = 0;
	TickType_t end = 0;
	for(;;)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
			start = xTaskGetTickCount();
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
				end = xTaskGetTickCount();
				if((end-start) > 3000 ){
					printf("Now a long press : %d\n",end - start);
					create.input = long_press;
				}
			}
			end = xTaskGetTickCount();
			if(end-start < 1000){
					create.input = single_press;
					printf("Now a single press : %d\n",end - start);
				}
		}
		else{
			create.input = no_press;
		}
		
		vTaskDelay(200); //DEBOUNCE
	}
}




//--------------------------------------------------------------------------------------------------------
//Main Program
//Controls all actions

int cSchedule = 0;
TickType_t currentTime = 0;
int currentTask = -1;

//helper method to choose scheduler
void switchSchedule(){
	allOff();
	if(cSchedule == 0){
		cSchedule++;
	}
	else{cSchedule--;}
}

//SELECTING SCHEDULER
void vSelectSchedule(void *pvParameters){
	
	create.input = no_press;
	for(;;)
	{
		while(selected == 0){
			if(cSchedule == 0){
				HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
				vTaskDelay(200);
				if(create.input == single_press){ 
					switchSchedule();
				}
				else if(create.input == long_press){
					selected = 1;
				}
			}//if cSchedule = 0
			
			if(cSchedule == 1){
				HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
				vTaskDelay(200);
				if(create.input == single_press){ 
					switchSchedule(); 
				}
				else if(create.input == long_press){
					selected = 2;
				}
			}// if cSchedule = 1
			
		}//while
		switch(selected){
			case 2:
				allON();
				vTaskDelay(1000);
				allOff();
			  vTaskResume(xExperiment);
				vTaskSuspend(NULL);
				break;
			case 1:
				allON();
				vTaskDelay(1000);
				allOff();
			  vTaskResume(xFPS);
			  vTaskSuspend(NULL);
				break;
		}
	}//for
}//vSelectScheduler

//check duration of coffee
bool checkDuration(int cT,int duration){
	return xTaskGetTickCount() - cT != duration;
}


	
//FIXED PRIORITY SCHEDULER
void vFPS(void *pvParameters){
	vTaskSuspend(NULL);
	printf("FPS Scheduler running");
	
	//sort by priority
	list[0] = espresso;
	list[3] = latte;
	list[2] = cappuccino;
	list[1] = mocha;
	
	currentTask = 0; // highest priority task
	currentTime = xTaskGetTickCount(); //t = 0
	int spin = 1; //Controls servo rotation
	
	
	Coffee cT; //coffee holder variable
	
	for(;;){
		cT = list[currentTask];
		while(checkDuration(currentTime,cT.duration*1000)){
				if(currentTask == 0){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
					vTaskResume(xEspresso);
				}//cT = 0;
				
				if(currentTask == 1){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
					vTaskResume(xCappuccino);
				}//cT = 1;
				
				if(currentTask == 2){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
					vTaskResume(xMocha);
				}//cT = 2;
					
				if(currentTask == 3){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
						vTaskResume(xLatte);
				}//cT = 3;
				
			}//while
		vTaskResume(xSoundHandler);
			//xSemaphoreGive(xBinarySemaphore);
		//GET NEXT TASK
			currentTime = xTaskGetTickCount();
			currentTask++;
			spin--;
			if(currentTask == 4){
				currentTask = 0;
			}
		}
	}



	//EXPERIMENTAL SCHEDULER: Dynamically extend the period
void vExperiment(void *pvParameters){
	vTaskSuspend(NULL);
	printf("Experimental Scheduler running");
	
	//Sort the list by priority
	//If the coffee have equal priority break tie with earliest deadline
	//sort by priority
	list[0] = espresso;
	list[3] = latte;
	list[1] = cappuccino;
	list[2] = mocha;
	
	currentTask = 0; // highest priority task
	currentTime = xTaskGetTickCount(); //t = 0
	int spin = 1; //Controls servo rotation
	
	
	Coffee cT; //coffee holder variable
	
	for(;;){
		cT = list[currentTask];
		while(checkDuration(currentTime,cT.duration*1000)){
				if(currentTask == 0){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
					vTaskResume(xEspresso);
				}//cT = 0;
				
				if(currentTask == 1){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
					vTaskResume(xCappuccino);
				}//cT = 1;
				
				if(currentTask == 2){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
					vTaskResume(xMocha);
				}//cT = 2;
					
				if(currentTask == 3){
					if(spin == 1){
						vTaskResume(xServo);
						spin++;
						list[currentTask].count++;
					}
						vTaskResume(xLatte);
				}//cT = 3;
				
			}//while
		vTaskResume(xSoundHandler);
		//GET NEXT TASK
			currentTime = xTaskGetTickCount();
			currentTask++;
			spin--;
			if(currentTask == 4){
				currentTask = 0;
			}
		}
}
	

//ESPRESSO TASK
void vEspresso(void *pvParameters){
	printf("Espresso running");
	vTaskSuspend(NULL);
	for(;;)
	{
		vTaskDelay(espresso.duration * 100);		
		//vTaskDelayUntil(&currentTime, espresso.duration * 100);
		vTaskSuspend(NULL);
	}
}

//LATTE TASK
void vLatte(void *pvParameters){
	printf("Latte running");
	vTaskSuspend(NULL);
	for(;;)
	{
		vTaskDelay(latte.duration * 100);
		vTaskSuspend(NULL);
	}
}

	
//CAPPUCCINO TASK
void vCappuccino(void *pvParameters){
	printf("Cappucino running");
	vTaskSuspend(NULL);
	for(;;)
	{
		vTaskDelay(cappuccino.duration * 100);
	}
}

//MOCHA TASK
void vMocha(void *pvParameters){
	printf("Mocha running");
	vTaskSuspend(NULL);
	for(;;)
	{
		vTaskDelay(mocha.duration * 100);
	}
}

//--------------------------------------------------------------------------------------------------------------------------------



//MUSIC


//--------------------------------------------------------------------------------------------------------------------


void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}

