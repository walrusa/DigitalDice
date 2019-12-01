#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "LCD.h"
#include "os.h"
#include "joystick.h"
#include "UART.h"
#include "FIFO.h"
#include "PORTE.h"
#include "tm4c123gh6pm.h"
#include "BSP.h"
// Constants
#define BGCOLOR     					LCD_BLACK
#define CROSSSIZE            			5

//------------------Defines and Variables-------------------
uint16_t origin[2]; // the original ADC value of x,y if the joystick is not touched
int16_t x = 63;  // horizontal position of the crosshair, initially 63
int16_t y = 63;  // vertical position of the crosshair, initially 63
int16_t z = 63;
int16_t prevx = 63;
int16_t	prevy = 63;
int16_t prev = 63;
uint8_t select;  // joystick push
char xstring[] = {'X', ':', ' ', 0x00};
char ystring[] = {'Y', ':', ' ', 0x00};
char zstring[] = {'Z', ':', ' ', 0x00};

//---------------------User debugging-----------------------

#define TEST_TIMER 0		// Change to 1 if testing the timer
#define TEST_PERIOD 2000000  // Defined by user
#define PERIOD 2000000  		// Defined by user

unsigned long Count;   		// number of times thread loops

//--------------------------------------------------------------
void CrossHair_Init(void){
	BSP_LCD_FillScreen(LCD_BLACK);	// Draw a black screen
	BSP_LCD_Message (1, 12, 3, xstring, x);
	BSP_LCD_Message (1, 12, 14, ystring, y);
	BSP_LCD_Message (0, 5, 3, zstring, z);
}

//******** Producer *************** 
void Producer(void){
#if TEST_TIMER
	PE1 ^= 0x02;	// heartbeat
	Count++;	// Increment dummy variable			
#else
	// Variable to hold updated x and y values
	rxDataType data;
	
	uint16_t rawX, rawY, rawZ; // To hold raw adc values
	BSP_Accelerometer_Input(&rawX, &rawY, &rawZ);
	
	data.x = rawX;
	data.y = rawY;
	data.z = rawZ;
	RxFifo_Put(data);
#endif
}

//******** Consumer *************** 
void Consumer(void){
	rxDataType data;
	RxFifo_Get(& data);
	x = data.x;
	y = data.y;
	z = data.z;

  printf("%d, %d, %d \n", x, y, z); 
	BSP_LCD_Message (1, 12, 3, xstring, x);
	BSP_LCD_Message (1, 12, 14, ystring, y);
	BSP_LCD_Message (0, 5, 3, zstring, z);
}

//******** Main *************** 
int main(void){
  PLL_Init(Bus80MHz);       // set system clock to 80 MHz
	Output_Init();
#if TEST_TIMER
	PortE_Init();       // profile user threads
	Count = 0;
	OS_AddPeriodicThread(&Producer, TEST_PERIOD, 1);
	while(1){}
#else

  	BSP_LCD_Init();        // initialize LCD
		printf("Starting\n");
	  BSP_Joystick_Init();   // initialize Joystick
  	CrossHair_Init();
		printf("Before\n");
		BSP_Accelerometer_Init();
		printf("After\n");		
		RxFifo_Init();
		OS_AddPeriodicThread(&Producer,PERIOD, 1);
		while(1){
			Consumer();
		}
#endif
} 
