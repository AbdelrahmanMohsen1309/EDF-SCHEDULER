/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"
/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

#define TASK1_PERIOD 50
#define TASK2_PERIOD 50 
#define TASK3_PERIOD 100
#define TASK4_PERIOD 20
#define TASK5_PERIOD 10
#define TASK6_PERIOD 100

#define BUTTON1_RISING_EDGE  'A'
#define BUTTON1_FALLING_EDGE 'B'
#define BUTTON2_RISING_EDGE  'C'
#define BUTTON2_FALLING_EDGE 'D'

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

QueueHandle_t xQueue1, xQueue2;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
pinState_t button_1_currentState = 0;
pinState_t button_1_nextState    = 0;
pinState_t button_2_currentState = 0;
pinState_t button_2_nextState    = 0;

/******************************************************************************
 *                            TASKS HANDLERS                                  *
 ******************************************************************************/
TaskHandle_t xTask1_Button1_Handle = NULL;
TaskHandle_t xTask2_Button2_Handle = NULL;
TaskHandle_t xTask3_Per_TX_Handle  = NULL;
TaskHandle_t xTask4_UART_RX_Handle = NULL;
TaskHandle_t xTask5_Load_1_Handle  = NULL;
TaskHandle_t xTask6_Load_2_Handle  = NULL;

/******************************************************************************
 *                                TASKS                                       *
 ******************************************************************************/
//--------------------------------------------------------------
// Task 1: ""Button_1_Monitor"", {Periodicity: 50, Deadline: 50}
// This task will monitor rising and falling edge on button 1 and 
// send this event to the consumer task. (Note: The rising and failling 
// edges are treated as separate events, hence they have separate strings)
//--------------------------------------------------------------
void Button_1_Monitor (void *pvParameters)
{
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = TASK1_PERIOD;
	unsigned char ulVar;
	while(1)
	{
		/*CLEAR ALL BITS AND SET PORT 1 PIN 1*/
		IOCLR1 |= 0x7C0000; IOSET1 |=0x020000;
		/*-----------------------------------*/
		
		button_1_currentState = button_1_nextState;
		button_1_nextState = GPIO_read(PORT_0,PIN1);
		
		if(button_1_currentState == 0 && button_1_nextState == 1)
		{
			ulVar = BUTTON1_RISING_EDGE;
			xQueueSend( xQueue1, ( void * ) &ulVar, ( TickType_t ) 1 );
		}
		else if(button_1_currentState == 1 && button_1_nextState == 0)
		{
			ulVar = BUTTON1_FALLING_EDGE;
			xQueueSend( xQueue1, ( void * ) &ulVar, ( TickType_t ) 1 );
		}
		/*CLEAR BIT BEFORE GOING TO BLOCKED STATE*/
	  IOCLR1 |= 0x020000;
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}

}
/******************************************************************************
 * Task 2: ""Button_2_Monitor"", {Periodicity: 50, Deadline: 50}							*
 * This task will monitor rising and falling edge on button 2 and send this 	*
 * event to the consumer task. (Note: The rising and failling edges are 			*
 * treated as separate events, hence they have separate strings)							*
 ******************************************************************************/
void Button_2_Monitor (void *pvParameters)
{
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = TASK2_PERIOD;
	unsigned char u2Var;
	while(1)
	{
		/*CLEAR ALL BITS AND SET PORT 1 PIN 2*/
		IOCLR1 |= 0x7A0000; IOSET1 |=0x040000;
		/*-----------------------------------*/
		
		button_2_currentState = button_2_nextState;
		button_2_nextState = GPIO_read(PORT_0,PIN2);
		
		if(button_2_currentState == 0 && button_2_nextState == 1)
		{
			u2Var = BUTTON2_RISING_EDGE;
			xQueueSend( xQueue1, ( void * ) &u2Var, ( TickType_t ) 1 );
		}
		else if(button_2_currentState == 1 && button_2_nextState == 0)
		{
			u2Var = BUTTON2_FALLING_EDGE;
			xQueueSend( xQueue1, ( void * ) &u2Var, ( TickType_t ) 1 );
		}
		
		IOCLR1 |= 0x040000;
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
		
	}
}

/******************************************************************************
 * Task 3: ""Periodic_Transmitter"", {Periodicity: 100, Deadline: 100}        *
 * This task will send preiodic string every 100ms to the consumer task       *
 ******************************************************************************/
void Periodic_Transmitter (void *pvParameters)
{
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = TASK3_PERIOD;
	unsigned char RXMessageFromButtons = 0;
	char * message;
	while(1)
	{
		/*CLEAR ALL BITS AND SET PORT 1 PIN 3*/
		IOCLR1 |= 0x760000; IOSET1 |=0x080000;
		/*-----------------------------------*/
		
		while( xQueueReceive( xQueue1, &( RXMessageFromButtons ),( TickType_t ) 2) == pdPASS )
		{
			if(RXMessageFromButtons == BUTTON1_RISING_EDGE)
			{
					message = "There is a Rising edge on Button 1";
					xQueueSend( xQueue2, ( void * ) &message, ( TickType_t ) 1 );
			}
			
			else if(RXMessageFromButtons == BUTTON1_FALLING_EDGE)
			{
					message = "There is a Falling edge on Button 1";
					xQueueSend( xQueue2, ( void * ) &message, ( TickType_t ) 1 );
			}
			
			else if(RXMessageFromButtons == BUTTON2_RISING_EDGE)
			{
					message = "There is a Rising edge on Button 2";
					xQueueSend( xQueue2, ( void * ) &message, ( TickType_t ) 1 );
			}
			
			else if(RXMessageFromButtons == BUTTON2_FALLING_EDGE)
			{
					message = "There is a Falling edge on Button 2";
					xQueueSend( xQueue2, ( void * ) &message, ( TickType_t ) 1 );
			}
			
		}
		IOCLR1 |= 0x080000;
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/******************************************************************************
 * Task 4: ""Uart_Receiver"", {Periodicity: 20, Deadline: 20}									*
 * This is the consumer task which will write on UART any received string     * 
 * from other tasks																														*
 ******************************************************************************/
void Uart_Receiver (void *pvParameters)
{
	TickType_t xLastWakeTime = 0;
	const TickType_t xFrequency = TASK4_PERIOD;
	char * RXMessageFromPeriodicTransmitterTask;
	int i = 0;
	while(1)
	{
		/*CLEAR ALL BITS AND SET PORT 1 PIN 4*/
		IOCLR1 |= 0x6E0000; IOSET1 |=0x100000;
		/*-----------------------------------*/
		
		while( xQueueReceive( xQueue2, &( RXMessageFromPeriodicTransmitterTask ),( TickType_t ) 2) == pdPASS )
		{
			//Loop to send all the string
			while(RXMessageFromPeriodicTransmitterTask[i] != '\0')
			{
				xSerialPutChar(RXMessageFromPeriodicTransmitterTask[i]);
				/*WAIT UART TO SEND BYTE AND U1LSR BIT 5 TO BE AN EMPTY*/
				while(!(U1LSR & 0x20));
				i++;
			}
			xSerialPutChar('\n');
			i = 0;
		}
		IOCLR1 |= 0x100000;
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/******************************************************************************
 * Task 5: ""Load_1_Simulation""                                              *
 * {Periodicity: 10, Deadline: 10}, Execution time: 5ms                       *
 ******************************************************************************/
void Load_1_Simulation (void *pvParameters)
{
	TickType_t xLastWakeTime = 0;
	int i;
	const TickType_t xFrequency = TASK5_PERIOD;
	while(1)
	{
		/*CLEAR ALL BITS AND SET PORT 1 PIN 1*/
		IOCLR1 |= 0x5E0000; IOSET1 |=0x200000;
		/*-----------------------------------*/
		
		
		for(i = 0 ; i < 33332 ; i++)
		{
			i=i;
		}
		
		IOCLR1 |= 0x200000;
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

/******************************************************************************
 * Task 6: ""Load_1_Simulation""                                              *
 * {Periodicity: 100, Deadline: 100}, Execution time: 12ms                    *
 ******************************************************************************/
void Load_2_Simulation (void *pvParameters)
{
	TickType_t xLastWakeTime = 0;
	int i;
	const TickType_t xFrequency = TASK6_PERIOD;
	while(1)
	{
		/*CLEAR ALL BITS AND SET PORT 1 PIN 6*/
		IOCLR1 |= 0x3E0000; IOSET1 |=0x400000;
		/*----------------------------------*/
		
		for(i = 0 ; i < 80000 ; i++)
		{
			i=i;
		}
		IOCLR1 |= 0x400000;
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

void vApplicationTickHook(void){
	TaskHandle_t crntTask = xTaskGetCurrentTaskHandle();
	GPIO_write(PORT_1,PIN0,PIN_IS_HIGH);
	GPIO_write(PORT_1,PIN0,PIN_IS_LOW);
	
	/*Spcify which task is currently work*/
	
	     if(crntTask == xTask1_Button1_Handle){IOCLR1 |= 0x7C0000; IOSET1 |=0x020000;}
	else if(crntTask == xTask2_Button2_Handle){IOCLR1 |= 0x7A0000; IOSET1 |=0x040000;}
	else if(crntTask == xTask3_Per_TX_Handle) {IOCLR1 |= 0x760000; IOSET1 |=0x080000;}
	else if(crntTask == xTask4_UART_RX_Handle){IOCLR1 |= 0x6E0000; IOSET1 |=0x100000;}
	else if(crntTask == xTask5_Load_1_Handle) {IOCLR1 |= 0x5E0000; IOSET1 |=0x200000;}
	else if(crntTask == xTask6_Load_2_Handle) {IOCLR1 |= 0x3E0000; IOSET1 |=0x400000;}
	else{IOCLR1 |= 0x7E0000;}

}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	/* Create a queue capable of containing 10 unsigned long values. */
  xQueue1 = xQueueCreate( 10, sizeof( unsigned char ) );
	
	xQueue2 = xQueueCreate( 10, sizeof( signed char * ) );
	
	xTaskPeriodicCreate(Button_1_Monitor,
												 "Button 1",
												 configMINIMAL_TASK_SIZE,
												 NULL,
												 1,
												 &xTask1_Button1_Handle,
												 TASK1_PERIOD);
												
	xTaskPeriodicCreate(Button_2_Monitor,
												 "Button 2",
												 configMINIMAL_TASK_SIZE,
												 NULL,
												 1,
												 &xTask2_Button2_Handle,
												 TASK2_PERIOD);
												 
	xTaskPeriodicCreate(Periodic_Transmitter,
												 "Transmitter",
												 configMINIMAL_TASK_SIZE,
												 NULL,
												 1,
												 &xTask3_Per_TX_Handle,
												 TASK3_PERIOD);
	
	xTaskPeriodicCreate(Uart_Receiver,
												 "UART Receiver",
												 configMINIMAL_TASK_SIZE,
												 NULL,
												 1,
												 &xTask4_UART_RX_Handle,
												 TASK4_PERIOD);
												 
	xTaskPeriodicCreate(Load_1_Simulation,
												 "Load 1",
												 configMINIMAL_TASK_SIZE,
												 NULL,
												 1,
												 &xTask5_Load_1_Handle,
												 TASK5_PERIOD);
	
	xTaskPeriodicCreate(Load_2_Simulation,
												 "Load 2",
												 configMINIMAL_TASK_SIZE,
												 NULL,
												 1,
												 &xTask6_Load_2_Handle,
												 TASK6_PERIOD);
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


