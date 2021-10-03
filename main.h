
/* FreeRTOS includes. */
#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"
#include "MY_CS43L22.h"




#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#define PI 3.14159f

//Sample rate and Output freq
#define F_SAMPLE		48000.0f // Sample freq
#define F_OUT				880.0f  // Output freq 
#define AMP				1000  // Output amplitute 

//button input
typedef enum {no_press, single_press, double_press, long_press} button_input;

//TONE STRUCTURE
/*Contains required values to generate tones*/
typedef struct{
	float mySinVal;
	float sample_dt;
	uint16_t sample_N;
	uint16_t i_t;
	uint32_t myDacVal;

	int16_t dataI2S[100];
} tone;

typedef struct Coffee{
	int duration;
	int deadline;
	int period;
	int count;
	int priority;
	int misses;
	bool run;
	TaskHandle_t handler;
	
}Coffee;

typedef struct Button{
	button_input input;
}Button;

extern Button create;

