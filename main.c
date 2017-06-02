/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * Demo Full project
 *
 * ArduinoMEGA with FreeRTOS 9.0.0
 *
 * Compiler: WinAVR
 * Burner: AVR Dude (STK500v2)
 * IDE: Eclipse Neon.3
 *
 * Description:
 * 	Implementation of the same Full Demo project available for Windows (MinGW).
 * 		Complete description and explanations in my blog http://hardwaresw.blogspot.de/
 * 	The code for Demo Blinky was kept (complete description in its own project available on https://github.com/brunolalb/FreeRTOS_ArduinoMEGA-Demo_Blinky
 * 									or in my blog http://hardwaresw.blogspot.de/2016/08/freertos-900-on-arduinomega-demo-blinky.html)
 * 	The code for Demo AVR323 was kept (complete description in its own project available on https://github.com/brunolalb/FreeRTOS_ArduinoMEGA-Demo_AVR323
 * 									or in my blog http://hardwaresw.blogspot.de/2016/08/freertos-900-on-arduinomega-demo-avr323.html)
 *
 * Initial version (2017-04-29): Bruno Landau Albrecht (brunolalb@gmail.com)
 *
 */


#include <stdlib.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h" /* Full demo */
#include "timers.h" /* Full demo */
#include "semphr.h"

/* Demo file headers. */
#include "partest.h"
#include "integer.h"
#include "serial.h"
#include "comtest.h"
#include "PollQ.h"
#include "RegTest/regtest.h"
#include "crflash.h"
#include "croutine.h"
/* Full Demo headers */
#include "error_messages.h"
#include "TaskNotify.h"
//#include "TimerDemo.h"

/* Priority definitions for most of the tasks in the demo application.  Some
tasks just use the idle priority. */
#define mainDEMOBLINKY_RECEIVE_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainDEMOBLINKY_SEND_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )

/* Baud rate used by the serial port tasks. */
#define mainCOM_TEST_BAUD_RATE				( ( uint32_t ) 115200 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainDEMOBLINKY_FREQUENCY_MS			( ( TickType_t ) (200) / portTICK_PERIOD_MS )

/* The period between executions of the check task. */
#define mainCHECK_PERIOD					( ( TickType_t ) 1000 / portTICK_PERIOD_MS  )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainDEMOBLINKY_QUEUE_LENGTH			( 1 )

/* Values passed to the two tasks just to check the task parameter
functionality. */
#define mainDEMOBLINKY_SEND_PARAMETER		( ( unsigned int ) 0x1111 )
#define mainDEMOBLINKY_RECEIVE_PARAMETER	( ( unsigned int ) 0x22 )

/* LED that is toggled every time the message is received for the Demo Blinky - Digital Pin 50 */
#define mainDEMOBLINKY_LED					( 3 )

/* LED used by the serial port tasks.  This is toggled on each character Tx,
and mainCOM_TEST_LED + 1 is toggles on each character Rx.  - Digital Pin 10 and 11 */
#define mainCOM_TEST_LED					( 4 )
#define mainCOM_TEST_LED1					( 5 )

/* LED that is toggled by the check task.  The check task periodically checks
that all the other tasks are operating without error.  If no errors are found
the LED is toggled.  If an error is found at any time the LED is never toggles
again. Digital Pin 12 */
#define mainCHECK_TASK_LED					( 6 )

/* LED that indicates an unexpected error (vAssertCalled) - Digital Pin 13*/
#define mainASSERTCALLED_LED				( 7 )

/* The number of coroutines to create. */
#define mainNUM_FLASH_COROUTINES			( 3 )

/* Flags to enable each functionality */
#define mainDEMO_BLINKY						0

#define mainDEMOAVR323						0
#if ( mainDEMOAVR323 == 1 )
#define mainDEMOAVR323_INTEGER				1
#define mainDEMOAVR323_COMTEST				1
#define mainDEMOAVR323_POLLEDQUEUE			1
#define mainDEMOAVR323_REGTEST				1
#define mainDEMOAVR323_ERRORCHECK			1
#define mainDEMOAVR323_COROUTINE			1
#endif

#define mainDEMOFULL						1
#if ( mainDEMOFULL == 1 )
#define mainDEMOFULL_NOTIFYTASK				1
#endif

/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
#if( mainDEMO_BLINKY == 1 )
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );
#endif

/*
 * The task function for the "Check" task.
 */
#if( mainDEMOAVR323_ERRORCHECK == 1 )
static void vErrorChecks( void *pvParameters );
#endif

/*
 * Checks the unique counts of other tasks to ensure they are still operational.
 * Flashes an LED if everything is okay.
 */
#if( mainDEMOAVR323_ERRORCHECK == 1 )
static void prvCheckOtherTasksAreStillRunning( void );
#endif

/*
 * Checks whether the tasks from the Full Demo are still running
 */
static void prvCheckTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The variable into which error messages are latched. */
static char pcStatusMessage[mainERRORSIZE];

/* The queue used by both tasks. */
#if( mainDEMO_BLINKY == 1 )
static QueueHandle_t xQueue = NULL;
#endif

/*-----------------------------------------------------------*/

int main( void )
{
	/* Setup the LED's for output. */
	vParTestInitialise();
	memcpy(pcStatusMessage,pcErrorOK,mainERRORSIZE);

#if( mainDEMO_BLINKY == 1 )
	/* Create the queue. */
	xQueue = xQueueCreate( mainDEMOBLINKY_QUEUE_LENGTH, sizeof( unsigned char ) );

	/* Demo Blinky tasks */
	if( xQueue != NULL )
	{
		/* Start the two tasks as described in the comments at the top of this file. */
		xTaskCreate( prvQueueReceiveTask,					/* The function that implements the task. */
					"RX", 									/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 				/* The size of the stack to allocate to the task. */
					( void * ) mainDEMOBLINKY_RECEIVE_PARAMETER, /* The parameter passed to the task - just to check the functionality. */
					mainDEMOBLINKY_RECEIVE_PRIORITY, 		/* The priority assigned to the task. */
					NULL );									/* The task handle is not required, so NULL is passed. */

		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, ( void * ) mainDEMOBLINKY_SEND_PARAMETER, mainDEMOBLINKY_SEND_PRIORITY, NULL );
	}
#endif

	/* Demo AVR323 tasks */
#if( mainDEMOAVR323 == 1 )
#if( mainDEMOAVR323_INTEGER == 1)
	vStartIntegerMathTasks( tskIDLE_PRIORITY );
#endif
#if( mainDEMOAVR323_COMTEST == 1)
	vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );
#endif
#if( mainDEMOAVR323_POLLEDQUEUE == 1 )
	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
#endif
#if( mainDEMOAVR323_REGTEST == 1 )
	vStartRegTestTasks();
#endif

#if( mainDEMOAVR323_ERRORCHECK == 1 )
	/* Create the tasks defined within this file. */
	xTaskCreate( vErrorChecks, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
#endif

#if( mainDEMOAVR323_COROUTINE == 1 )
	/* Create the co-routines that flash the LED's. */
	vStartFlashCoRoutines( mainNUM_FLASH_COROUTINES );
#endif
#endif //mainDEMOAVR323

#if ( mainDEMOFULL == 1 )
	/* Start the check task as described at the top of this file. */
	xTaskCreate( prvCheckTask, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
#if ( mainDEMOFULL_NOTIFYTASK == 1 )
	vStartTaskNotifyTask();
#endif



#endif

	/* In this port, to use preemptive scheduler define configUSE_PREEMPTION
	as 1 in portmacro.h.  To use the cooperative scheduler define
	configUSE_PREEMPTION as 0. */
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

#if( mainDEMO_BLINKY == 1 )
static void prvQueueSendTask( void *pvParameters )
{
TickType_t xNextWakeTime;
const unsigned char ucValueToSend = 100;
const TickType_t xBlockTime = pdMS_TO_TICKS( mainDEMOBLINKY_FREQUENCY_MS );

	/* Remove compiler warning in the case that configASSERT() is not
	defined. */
	( void ) pvParameters;

	/* Check the task parameter is as expected. */
	configASSERT( ( ( unsigned int ) pvParameters ) == mainDEMOBLINKY_SEND_PARAMETER );

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again.
		The block time is specified in ticks, the constant used converts ticks
		to ms.  While in the Blocked state this task will not consume any CPU
		time. */
		vTaskDelayUntil( &xNextWakeTime, xBlockTime );

		/* Send to the queue - causing the queue receive task to unblock and
		toggle the LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		xQueueSend( xQueue, &ucValueToSend, 0U );
	}
}
#endif
/*-----------------------------------------------------------*/

#if( mainDEMO_BLINKY == 1 )
static void prvQueueReceiveTask( void *pvParameters )
{
unsigned char ucReceivedValue;

	/* Remove compiler warning in the case that configASSERT() is not
	defined. */
	( void ) pvParameters;

	/* Check the task parameter is as expected. */
	configASSERT( ( ( unsigned int ) pvParameters ) == mainDEMOBLINKY_RECEIVE_PARAMETER );

	for( ;; )
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
		xQueueReceive( xQueue, &ucReceivedValue, portMAX_DELAY );

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, toggle the LED. */
		if( ucReceivedValue == 100UL )
		{
			/* The Windows Blinky Demo prints a message with printf().
			 * In Arduino, we'll just blink a LED */
			vParTestToggleLED(mainDEMOBLINKY_LED);

			ucReceivedValue = 0U;
		}
	}
}
#endif
/*-----------------------------------------------------------*/

#if( mainDEMOAVR323_ERRORCHECK == 1 )
static void vErrorChecks( void *pvParameters )
{
static volatile uint32_t ulDummyVariable = 3UL;

	/* The parameters are not used. */
	( void ) pvParameters;

	/* Cycle for ever, delaying then checking all the other tasks are still
	operating without error. */
	for( ;; )
	{
		vTaskDelay( mainCHECK_PERIOD );

		/* Perform a bit of 32bit maths to ensure the registers used by the
		integer tasks get some exercise. The result here is not important -
		see the demo application documentation for more info. */
		ulDummyVariable *= 3;

		prvCheckOtherTasksAreStillRunning();
	}
}
#endif

/*-----------------------------------------------------------*/
#if ( mainDEMOAVR323_ERRORCHECK == 1 )
static void prvCheckOtherTasksAreStillRunning( void )
{
static portBASE_TYPE xErrorHasOccurred = pdFALSE;

#if( mainDEMOAVR323_INTEGER == 1)
	if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
	{
		xErrorHasOccurred = pdTRUE;
	}
#endif

#if( mainDEMOAVR323_COMTEST == 1 )
	if( xAreComTestTasksStillRunning() != pdTRUE )
	{
		xErrorHasOccurred = pdTRUE;
	}
#endif

#if( mainDEMOAVR323_POLLEDQUEUE == 1 )
	if( xArePollingQueuesStillRunning() != pdTRUE )
	{
		xErrorHasOccurred = pdTRUE;
	}
#endif

#if( mainDEMOAVR323_REGTEST == 1 )
	if( xAreRegTestTasksStillRunning() != pdTRUE )
	{
		xErrorHasOccurred = pdTRUE;
	}
#endif

	if( xErrorHasOccurred == pdFALSE )
	{
		/* Toggle the LED if everything is okay so we know if an error occurs even if not
		using console IO. */
		vParTestToggleLED( mainCHECK_TASK_LED );
	}
}
#endif
/*-----------------------------------------------------------*/

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
	/* Parameters are not used. */
	( void ) ulLine;
	( void ) pcFileName;

	vParTestToggleLED(mainASSERTCALLED_LED);
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{

#if ( mainDEMOFULL == 1 )
#if ( 0 == 1 )
	/* Call the periodic timer test, which tests the timer API functions that
	can be called from an ISR. */
	#if( configUSE_PREEMPTION != 0 )
	{
		/* Only created when preemption is used. */
		vTimerPeriodicISRTests();
	}
	#endif

	/* Call the periodic queue overwrite from ISR demo. */
	vQueueOverwritePeriodicISRDemo();

	/* Write to a queue that is in use as part of the queue set demo to
	demonstrate using queue sets from an ISR. */
	vQueueSetAccessQueueSetFromISR();
	vQueueSetPollingInterruptAccess();

	/* Exercise event groups from interrupts. */
	vPeriodicEventGroupsProcessing();

	/* Exercise giving mutexes from an interrupt. */
	vInterruptSemaphorePeriodicTest();
#endif

#if ( mainDEMOFULL_NOTIFYTASK == 1 )
	/* Exercise using task notifications from an interrupt. */
	xNotifyTaskFromISR();
#endif

#endif
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
#if( mainDEMO_COROUTINE == 1 )
	vCoRoutineSchedule();
#endif
}

/*-----------------------------------------------------------*/

static void prvCheckTask( void *pvParameters )
{
TickType_t xNextWakeTime;
const TickType_t xCycleFrequency = pdMS_TO_TICKS( 2500UL );
BaseType_t xCheckResult;

	/* Just to remove compiler warning. */
	( void ) pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
#if ( mainDEMOFULL == 1 )
	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again. */
		vTaskDelayUntil( &xNextWakeTime, xCycleFrequency );

		/* Initialize the result variable every cycle */
		xCheckResult = pdTRUE;

#if ( mainDEMOFULL_TIMERDEMO == 1 )
		/* Check the standard demo tasks are running without error. */
		#if( configUSE_PREEMPTION != 0 )
		{
			/* These tasks are only created when preemption is used. */
			if( xAreTimerDemoTasksStillRunning( xCycleFrequency ) != pdTRUE )
			{
				pcStatusMessage = "Error: TimerDemo";
				xCheckResult = pdFAIL;
			}
		}
		#endif
#endif

#if ( mainDEMOFULL_NOTIFYTASK == 1 )
		if( xAreTaskNotificationTasksStillRunning() != pdTRUE )
		{
			memcpy(pcStatusMessage, pcErrorNotification, mainERRORSIZE);
			xCheckResult = pdFAIL;
		}
#endif

#if ( 0 == 1)
		if( xAreInterruptSemaphoreTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "Error: IntSem";
			xCheckResult = pdFAIL;
		}
		else if( xAreEventGroupTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "Error: EventGroup";
			xCheckResult = pdFAIL;
		}
	    else if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
	    {
			pcStatusMessage = "Error: IntMath";
			xCheckResult = pdFAIL;
	    }
		else if( xAreGenericQueueTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "Error: GenQueue";
			xCheckResult = pdFAIL;
		}
		else if( xAreQueuePeekTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "Error: QueuePeek";
			xCheckResult = pdFAIL;
		}
		else if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			pcStatusMessage = "Error: BlockQueue";
			xCheckResult = pdFAIL;
		}
	    else if( xAreSemaphoreTasksStillRunning() != pdTRUE )
	    {
			pcStatusMessage = "Error: SemTest";
			xCheckResult = pdFAIL;
	    }
	    else if( xArePollingQueuesStillRunning() != pdTRUE )
	    {
			pcStatusMessage = "Error: PollQueue";
			xCheckResult = pdFAIL;
	    }
		else if( xAreMathsTaskStillRunning() != pdPASS )
		{
			pcStatusMessage = "Error: Flop";
			xCheckResult = pdFAIL;
		}
	    else if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
	    {
			pcStatusMessage = "Error: RecMutex";
			xCheckResult = pdFAIL;
		}
		else if( xAreCountingSemaphoreTasksStillRunning() != pdTRUE )
		{
			pcStatusMessage = "Error: CountSem";
			xCheckResult = pdFAIL;
		}
		else if( xIsCreateTaskStillRunning() != pdTRUE )
		{
			pcStatusMessage = "Error: Death";
			xCheckResult = pdFAIL;
		}
		else if( xAreDynamicPriorityTasksStillRunning() != pdPASS )
		{
			pcStatusMessage = "Error: Dynamic";
			xCheckResult = pdFAIL;
		}
		else if( xAreQueueSetTasksStillRunning() != pdPASS )
		{
			pcStatusMessage = "Error: Queue set";
			xCheckResult = pdFAIL;
		}
		else if( xIsQueueOverwriteTaskStillRunning() != pdPASS )
		{
			pcStatusMessage = "Error: Queue overwrite";
			xCheckResult = pdFAIL;
		}
		else if( xAreQueueSetPollTasksStillRunning() != pdPASS )
		{
			pcStatusMessage = "Error: Queue set polling";
			xCheckResult = pdFAIL;
		}
		else if( xAreBlockTimeTestTasksStillRunning() != pdPASS )
		{
			pcStatusMessage = "Error: Block time";
			xCheckResult = pdFAIL;
		}
		else if( xAreAbortDelayTestTasksStillRunning() != pdPASS )
		{
			pcStatusMessage = "Error: Abort delay";
			xCheckResult = pdFAIL;
		}
#endif

		if ( xCheckResult == pdFAIL)
		{
			vParTestToggleLED(mainCHECK_TASK_LED);
		}

		/*
		 * Maybe I should implement some way of send the message through a serial connection
		 */
	}
#endif
}

