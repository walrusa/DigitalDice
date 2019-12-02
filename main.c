// Main.c
// Runs on LM4F120/TM4C123
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
// Modified by Sile Shu 10/4/17, ss5de@virginia.edu
// Modified by Mustafa Hotaki 7/29/18, mkh3cf@virginia.edu

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include <string.h> 
#include "UART.h"
#include "FIFO.h"
#include "joystick.h"
#include "PORTE.h"
#include "BSP.h"
#include <math.h>
#include <time.h> 
// Constants
#define BGCOLOR     					LCD_BLACK
#define CROSSSIZE            	5
#define PERIOD               	4000000000   // DAS 20Hz sampling period in system time units
#define PSEUDOPERIOD         	8000000
#define LIFETIME             	1000
#define RUNLENGTH            	600 // 30 seconds run length

Sema4Type LCDFree;
uint16_t origin[2]; 	// The original ADC value of x,y if the joystick is not touched, used as reference
int16_t x = 63;  			// horizontal position of the crosshair, initially 63
int16_t y = 63;  			// vertical position of the crosshair, initially 63
int16_t z = 63;
int16_t prevx, prevy;	// Previous x and y values of the crosshair
uint8_t select;  			// joystick push
uint8_t area[2];
uint32_t PseudoCount;

uint16_t xdata[16] = {0};
uint16_t ydata[16] = {0};
uint16_t zdata[16] = {0};
uint32_t currentXSum;
uint32_t currentYSum;
uint32_t currentZSum;
uint16_t currentIndex;
uint16_t currentXAverage;
uint16_t currentYAverage;
uint16_t currentZAverage;
uint16_t dataPoints = 0;
uint16_t currentRoll = 1;
char xstring[] = {'X', ':', 0x00};
char ystring[] = {'Y', ':', 0x00};
char zstring[] = {'Z', ':', 0x00};
char loading[] = {'L', 'O', 'A', 'D', 'I', 'N', 'G'};
char roll[] = {0x00};
char sideArr[] = {0x00};
char sideNameArr[] = {0x00};

uint8_t steadySamples;
uint8_t rollingSamples;
uint8_t done;
uint8_t sides;
uint8_t inDiceMode;
uint8_t inControlMode;
uint8_t killThreads;

void controlScreen(void);
void bufferState(void);

unsigned long NumCreated;   		// Number of foreground threads created
unsigned long NumSamples;   		// Incremented every ADC sample, in Producer
unsigned long UpdateWork;   		// Incremented every update on position values
unsigned long Calculation;  		// Incremented every cube number calculation

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
unsigned long TotalWithI1;
unsigned short MaxWithI1;

void Device_Init(void){
	BSP_LCD_Init();
	BSP_Joystick_Init();
	BSP_LCD_FillScreen(LCD_BLACK);
	BSP_Accelerometer_Init();
	Output_Init();

}
//------------------Task 1--------------------------------
// background thread executed at 20 Hz
//******** Producer *************** 
int UpdatePosition(uint16_t rawx, uint16_t rawy, jsDataType* data){
	if (rawx > origin[0]){
		x = x + ((rawx - origin[0]) >> 9);
	}
	else{
		x = x - ((origin[0] - rawx) >> 9);
	}
	if (rawy < origin[1]){
		y = y + ((origin[1] - rawy) >> 9);
	}
	else{
		y = y - ((rawy - origin[1]) >> 9);
	}
	if (x > 127){
		x = 127;}
	if (x < 0){
		x = 0;}
	if (y > 120 - CROSSSIZE){
		y = 120 - CROSSSIZE;}
	if (y < 0){
		y = 0;}
	data->x = x; data->y = y;
	return 1;
}

void Producer(void){
//	uint16_t rawX,rawY; // raw adc value
//	uint8_t select;
//	jsDataType data;
//	unsigned static long LastTime;  // time at previous ADC sample
//	unsigned long thisTime;         // time at current ADC sample
//	long jitter;                    // time between measured and expected, in us
//	if (NumSamples < RUNLENGTH){
//		BSP_Joystick_Input(&rawX,&rawY,&select);
//		thisTime = OS_Time();       // current time, 12.5 ns
//		UpdateWork += UpdatePosition(rawX,rawY,&data); // calculation work
//		NumSamples++;               // number of samples
//		if(JsFifo_Put(data) == 0){ // send to consumer
//			DataLost++;
//		}
//	//calculate jitter
//		if(UpdateWork > 1){    // ignore timing of first interrupt
//			unsigned long diff = OS_TimeDifference(LastTime,thisTime);
//			if(diff > PERIOD){
//				jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
//			}
//			else{
//				jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
//			}
//			if(jitter > MaxJitter){
//				MaxJitter = jitter; // in usec
//			}       // jitter should be 0
//			if(jitter >= JitterSize){
//				jitter = JITTERSIZE-1;
//			}
//			JitterHistogram[jitter]++; 
//		}
//		LastTime = thisTime;
//	}
while (1) {
	jsDataType data;
	
	uint16_t rawX, rawY, rawZ; // To hold raw adc values
	BSP_Accelerometer_Input(&rawX, &rawY, &rawZ);
	
	data.x = rawX;
	data.y = rawY;
	data.z = rawZ;
	JsFifo_Put(data);
	if (!inDiceMode){ NumCreated--; OS_Kill(); }
	OS_Suspend();
}

}

//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 1 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
	uint32_t StartTime,CurrentTime,ElapsedTime;
	StartTime = OS_MsTime();
	ElapsedTime = 0;
	OS_bWait(&LCDFree);
	BSP_LCD_FillScreen(BGCOLOR);
	while (ElapsedTime < LIFETIME){

		CurrentTime = OS_MsTime();
		ElapsedTime = CurrentTime - StartTime;
		BSP_LCD_Message(0,5,0,"Life Time:",LIFETIME);
		BSP_LCD_Message(1,0,0,"Horizontal Area:",area[0]);
		BSP_LCD_Message(1,1,0,"Vertical Area:",area[1]);
		BSP_LCD_Message(1,2,0,"Elapsed Time:",ElapsedTime);
		OS_Sleep(50);
	}
	BSP_LCD_FillScreen(BGCOLOR);
	OS_bSignal(&LCDFree);
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
  if(OS_MsTime() > 20 ){ // debounce
    if(OS_AddThread(&ButtonWork,128,4)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------

//******** Consumer *************** 
// foreground thread, accepts data from producer
// Display crosshair and its positions
// inputs:  none
// outputs: none
void Consumer(void){

//	while(NumSamples < RUNLENGTH){
//		jsDataType data;
//		JsFifo_Get(&data);
//		OS_bWait(&LCDFree);
//			
//		BSP_LCD_DrawCrosshair(prevx, prevy, LCD_BLACK); // Draw a black crosshair
//		BSP_LCD_DrawCrosshair(data.x, data.y, LCD_RED); // Draw a red crosshair

//		BSP_LCD_Message(1, 5, 3, "X: ", x);		
//		BSP_LCD_Message(1, 5, 12, "Y: ", y);
//		OS_bSignal(&LCDFree);
//		prevx = data.x; 
//		prevy = data.y;
//	}
	while (inDiceMode) {
	jsDataType data;
	JsFifo_Get(& data);
	x = data.x;
	y = data.y;
	z = data.z;
	OS_bWait(&LCDFree);
	if (done == 0) {
	if (dataPoints < 100) {
		BSP_LCD_DrawString(0 , 0 , "Dice mode activated" , 0x07E0);
		if ((dataPoints & 8) == 0) {
				BSP_LCD_DrawString(7, 6, loading, 0xFFFF);
		}
		else if ((dataPoints & 8) == 8) {
				BSP_LCD_DrawString(7, 6, loading, 0x0000);
		}
		dataPoints++;
	}
	else if (dataPoints < 116) {
		xdata[currentIndex] = x;
		ydata[currentIndex] = y;
		zdata[currentIndex] = z;
		currentXSum += xdata[currentIndex];
		currentYSum += ydata[currentIndex];
		currentZSum += zdata[currentIndex];
		currentIndex++;
		if (currentIndex > 15) {
			currentIndex = 0;
		}
		dataPoints++;
		if ((dataPoints & 8) == 8) {
				BSP_LCD_DrawString(7, 6, loading, 0xFFFF);
		}
		else  if ((dataPoints & 8) == 8) {
				BSP_LCD_DrawString(7, 6, loading, 0x0000);
		}
	}
	else if (dataPoints == 116) {
		BSP_LCD_DrawString(0 , 0 , "Dice mode activated" , 0x0000);
		BSP_LCD_DrawString(7, 6, loading, 0x0000);
		currentXSum -= xdata[currentIndex];
		currentYSum -= ydata[currentIndex];
		currentZSum -= zdata[currentIndex];
		xdata[currentIndex] = x;
		ydata[currentIndex] = y;
		zdata[currentIndex] = z;
		currentXSum += xdata[currentIndex];
		currentYSum += ydata[currentIndex];
		currentZSum += zdata[currentIndex];
		currentIndex++;
		if (currentIndex > 15) {
			currentIndex = 0;
		}
		currentXAverage = currentXSum >> 4;
		currentYAverage = currentYSum >> 4;
		currentZAverage = currentZSum >> 4;
		
		if (abs(x - currentXAverage) < 30 && abs(y - currentYAverage) < 30 && abs(z - currentZAverage) < 30) {
				printf("%d, %d, %d, %d, %d, %d, %d \n", x, y, z, currentRoll, currentXAverage, currentYAverage, currentZAverage);
				BSP_LCD_MessageBig (0, 4, 7, roll, currentRoll, 8);
		}
		else {
				currentRoll = rand() % sides + 1;
				printf("%d, %d, %d, %d, %d, %d, %d \n", x, y, z, currentRoll, currentXAverage, currentYAverage, currentZAverage);
				BSP_LCD_MessageBig (0, 4, 7, roll, currentRoll, 8);

		}
		dataPoints++;
	}
	else {
		currentXSum -= xdata[currentIndex];
		currentYSum -= ydata[currentIndex];
		currentZSum -= zdata[currentIndex];
		xdata[currentIndex] = x;
		ydata[currentIndex] = y;
		zdata[currentIndex] = z;
		currentXSum += xdata[currentIndex];
		currentYSum += ydata[currentIndex];
		currentZSum += zdata[currentIndex];
		currentIndex++;
		if (currentIndex > 15) {
			currentIndex = 0;
		}
		currentXAverage = currentXSum >> 4;
		currentYAverage = currentYSum >> 4;
		currentZAverage = currentZSum >> 4;
		
		if (abs(x - currentXAverage) < 30 && abs(y - currentYAverage) < 30 && abs(z - currentZAverage) < 30) {
				steadySamples++;
				printf("%d, %d, %d, %d, %d, %d, %d \n", x, y, z, currentRoll, currentXAverage, currentYAverage, currentZAverage);
				BSP_LCD_MessageBig (0, 4, 7, roll, currentRoll, 8);
		}
		else {
				rollingSamples++;
				steadySamples = 0;
				currentRoll = rand() % sides + 1;
				printf("%d, %d, %d, %d, %d, %d, %d \n", x, y, z, currentRoll, currentXAverage, currentYAverage, currentZAverage);
				BSP_LCD_MessageBig (0, 4, 7, roll, currentRoll, 8);

		}
		dataPoints++;
	}
}
	//BSP_LCD_Message (1, 12, 1, xstring, x);
	//BSP_LCD_Message (1, 12, 8, ystring, y);
	//BSP_LCD_Message (1, 12, 15, zstring, z);
	OS_bSignal(&LCDFree);
	if (steadySamples > 20 && rollingSamples > 0) {
		done = 1;
	}
	if (!inDiceMode){ NumCreated--; OS_Kill(); }}
		if (!inDiceMode){ NumCreated--; OS_Kill(); }
  //OS_Kill();  // done
}


//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
void incrementSides(void){
		
	   sides++;
	   if (sides == 10){ 
			sides = 2; 
			}
		 //BSP_LCD_Message(1,3,4,"Sides: " , sides);
		 BSP_LCD_MessageBig(0,7,8,sideArr , sides, 6);

	
}

void exitDiceMode(void){
	
	inDiceMode = 0;
	inControlMode = 0;
	killThreads = 1;
	NumCreated += OS_AddThread(&bufferState , 128 ,  1);
	done = 0;
	
}

void reset(void){
	dataPoints = 0;
	currentIndex = 0;
	currentXSum = 0;
	currentYSum = 0;
	currentZSum = 0;
	currentRoll = 1;
	steadySamples = 0;
	rollingSamples = 0;
	done = 0;
}

void enterDiceMode(void){

	volatile unsigned long i;
		
	inDiceMode = 1;
	inControlMode = 0;
	dataPoints = 0;
	currentIndex = 0;
	currentXSum = 0;
	currentYSum = 0;
	currentZSum = 0;
	steadySamples = 0;
	rollingSamples = 0;
	done = 0;
	
	OS_bWait(&LCDFree);

	BSP_LCD_FillScreen(0);
	//BSP_LCD_DrawString(0 , 0 , "Dice mode activated" , 0x07E0);
	
	OS_bSignal(&LCDFree);
	//OS_AddPeriodicThread(&Producer,PERIOD,1); // 2 kHz real time sampling of PD3
	//NumCreated += OS_AddThread(&Interpreter, 128,2); 
  NumCreated += OS_AddThread(&Consumer, 128,1);
	NumCreated += OS_AddThread(&Producer, 128,1);

	
	OS_AddSW1Task(&exitDiceMode , 1);
	OS_AddSW2Task(&reset , 1);

}

void enterBufferState(void){
	inControlMode = 0;
	inDiceMode = 0;
	NumCreated += OS_AddThread(&bufferState , 128 , 1);
}

void controlScreen(void){
	OS_bWait(&LCDFree);
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_LCD_DrawString(0 , 0 , "Button1: " , 0x07E0);
	BSP_LCD_DrawString(2 , 1 , "Increment sides" , 0x07E0);
	BSP_LCD_DrawString(0 , 2 , "Button2: " , 0x07E0);
	BSP_LCD_DrawString(2 , 3 , "Ret to buffer state" , 0x07E0);	
	BSP_LCD_DrawString(7, 5, "(Sides)", LCD_CYAN);
	BSP_LCD_MessageBig(0,7,8,sideArr , sides, 6);
	OS_bSignal(&LCDFree);
	OS_AddSW1Task(&incrementSides , 1);
	OS_AddSW2Task(&enterBufferState , 1);


	while(1){	
		if (inDiceMode || !inControlMode){ 
			NumCreated--; OS_Kill(); 
		}
	}
	    
}

void enterControlMode(void){
	NumCreated += OS_AddThread(&controlScreen , 128 , 1);
	inControlMode = 1;
	inDiceMode = 0;
}


void bufferState(void){
	

	OS_bWait(&LCDFree);
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_LCD_DrawString(0 , 0 , "Button1: " , 0x07E0);
	BSP_LCD_DrawString(2 , 1 , "Enter dice mode" , 0x07E0);
	BSP_LCD_DrawString(0 , 2 , "Button2: " , 0x07E0);
  BSP_LCD_DrawString(2 , 3 , "Enter edit mode" , 0x07E0);	
	OS_bSignal(&LCDFree);
	OS_AddSW1Task(&enterDiceMode , 1);
	OS_AddSW2Task(&enterControlMode , 1);
	

	while (1){	
		if (inControlMode || inDiceMode){ 
			NumCreated--; 	
			OS_Kill(); 
		}
	}	
}

//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------
void drawCircle(int p1, int p2 , int radius){
	int x,y,i;
	for(x=(p1-radius);x<=(p1+radius);x++)
	{
		y = sqrt(radius*radius - pow((x-p1),2)) + p2;
		BSP_LCD_DrawPixel(x,y,0x0000);
		for (i = p2; i < y; i++){
		  BSP_LCD_DrawPixel(x , i , 0x0000);
		}
	}
	for(x=(p1-radius);x<=(p1+radius);x++)
	{
		y = p2 - sqrt(radius*radius - pow((x-p1),2));
		BSP_LCD_DrawPixel(x,y,0x0000);
		for (i = p2; i > y; i--){
		  BSP_LCD_DrawPixel(x , i , 0x0000);
	  }
	}
	
}

void startScreen(void){
				int32_t wait = 0;
				OS_bWait(&LCDFree);
		    OS_ClearMsTime();
        BSP_LCD_FillScreen(BGCOLOR);
				BSP_LCD_DrawString(5 , 0 , "Digital Dice" , 0x07E0); 
        OS_ClearMsTime();	
	
        while (wait < 3){
					BSP_LCD_FillRect(32,32,64,64, 0xFFFF);	
					drawCircle(47 , 47 , 10);	
					drawCircle(81 , 81 , 10);
					while (OS_MsTime() < 1000){  }	
					OS_ClearMsTime();
					BSP_LCD_FillRect(32,32,64,64, 0x0000);	
					while (OS_MsTime() < 1000){}
          OS_ClearMsTime();
          wait++;						
        }
				
				//BSP_LCD_FillScreen(BGCOLOR);
        //BSP_LCD_DrawString(0 , 0 , "Done" , 0x07E0);
				//OS_ClearMsTime();
				//while (OS_MsTime() < 1000){}
				
				inDiceMode = 0;
				inControlMode = 0;
				sides = 6;
				NumCreated += OS_AddThread(&bufferState , 128 , 1);
				NumCreated--;
				OS_bSignal(&LCDFree);
				OS_Kill();       				
}

//--------------end of Task 5-----------------------------


//******************* Main Function**********
int main(void){ 
  OS_Init();           // initialize, disable interrupts
	Device_Init();
  //CrossHair_Init();
	currentIndex = 0;
	currentXSum = 0;
	currentYSum = 0;
	currentZSum = 0;

  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  MaxJitter = 0;       // in 1us units

//********initialize communication channels
  JsFifo_Init();
	srand(100); 

//*******attach background tasks***********
  //OS_AddSW1Task(&SW1Push,2);
  //OS_AddPeriodicThread(&Producer,PERIOD,1); // 2 kHz real time sampling of PD3
	
  NumCreated = 0 ;
// create initial foreground threads
  //NumCreated += OS_AddThread(&Interpreter, 128,2); 
  //NumCreated += OS_AddThread(&Consumer, 128,1); 
	 //NumCreated += OS_AddThread(&Producer, 128,1);
	//NumCreated += OS_AddThread(&CubeNumCalc, 128,1);
   NumCreated += OS_AddThread(&startScreen , 128 , 1); 
	OS_InitSemaphore(&LCDFree, 1);
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
	return 0;            // this never executes
}
