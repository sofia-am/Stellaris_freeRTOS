/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*
 * This project contains an application demonstrating the use of the
 * FreeRTOS.org mini real time scheduler on the Luminary Micro LM3S811 Eval
 * board.  See http://www.FreeRTOS.org for more information.
 *
 * main() simply sets up the hardware, creates all the demo application tasks,
 * then starts the scheduler.  http://www.freertos.org/a00102.html provides
 * more information on the standard demo tasks.
 *
 * In addition to a subset of the standard demo application tasks, main.c also
 * defines the following tasks:
 *
 * + A 'Print' task.  The print task is the only task permitted to access the
 * LCD - thus ensuring mutual exclusion and consistent access to the resource.
 * Other tasks do not access the LCD directly, but instead send the text they
 * wish to display to the print task.  The print task spends most of its time
 * blocked - only waking when a message is queued for display.
 *
 * + A 'Button handler' task.  The eval board contains a user push button that
 * is configured to generate interrupts.  The interrupt handler uses a
 * semaphore to wake the button handler task - demonstrating how the priority
 * mechanism can be used to defer interrupt processing to the task level.  The
 * button handler task sends a message both to the LCD (via the print task) and
 * the UART where it can be viewed using a dumb terminal (via the UART to USB
 * converter on the eval board).  NOTES:  The dumb terminal must be closed in
 * order to reflash the microcontroller.  A very basic interrupt driven UART
 * driver is used that does not use the FIFO.  19200 baud is used.
 *
 * + A 'check' task.  The check task only executes every five seconds but has a
 * high priority so is guaranteed to get processor time.  Its function is to
 * check that all the other tasks are still operational and that no errors have
 * been detected at any time.  If no errors have every been detected 'PASS' is
 * written to the display (via the print task) - if an error has ever been
 * detected the message is changed to 'FAIL'.  The position of the message is
 * changed for each write.
 */

/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Demo app includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"

#define mainQUEUE_TIMEOUT 3000
#define mainFILTER_TIMEOUT 1000
#define mainLCD_TIMEOUT 1000
#define mainUART_TIMEOUT 3000

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY ((TickType_t)5000 / portTICK_PERIOD_MS)

/* Delay between cycles of the 'sensor' task. (10 Hz -> 0,1 seg)*/
#define mainSENSOR_DELAY ((TickType_t)100 / portTICK_PERIOD_MS)

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE (19200)
#define mainFIFO_SET (0x10)

/* Demo task priorities. */
#define mainSENSOR_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define mainFILTER_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define mainDISPLAY_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define mainUART_TASK_PRIORITY (tskIDLE_PRIORITY + 4)

/* Misc. */
#define mainQUEUE_SIZE (3)
#define mainNO_DELAY ((TickType_t)0)

#define MAX_N 15

#define UART_BUFFER_SIZE 5
/*
 * Configure the processor and peripherals for this demo.
 */
static void prvSetupHardware(void);

/*
 * The 'check' task, as described at the top of this file.
 */
static void vCheckTask(void *pvParameters);

/*
 * The task that is woken by the ISR that processes GPIO interrupts originating
 * from the push button.
 */
static void vButtonHandlerTask(void *pvParameters);

/*
 * The task that controls access to the LCD.
 */
static void vPrintTask(void *pvParameter);

/*
 * The task that is woken by the ISR that processes UART interrupts.
 */
static void vUARTIntHandler(void);

/*
 * The task that simulates a sensor.
 */
static void vSensorTask(void *pvParameters);
/*
 * The task that simulates a sensor.
 */
static void vFilterTask(void *pvParameters);

/* String that is transmitted on the UART. */
static char *cMessage = "Task woken by button interrupt! --- ";
static volatile char *pcNextChar;

/* Number of samples taken by the filter */
static uint8_t N;

static int temperature = 0;

int8_t sampledData[MAX_N];

/* The semaphore used to wake the button handler task from within the GPIO
interrupt handler. */
SemaphoreHandle_t xUARTSemaphore;

/* The semaphore used to change the value of the N parameter */
SemaphoreHandle_t xFilterSemaphore;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xPrintQueue;

/* The queue used to send temperature data to the filter. */
QueueHandle_t xSensorQueue;

/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the semaphore used to change variable N */
	vSemaphoreCreateBinary(xFilterSemaphore);
	// xSemaphoreTake(xFilterSemaphore, 0);
	/* Create the sempahore to wake up to uart handler*/
	vSemaphoreCreateBinary(xUARTSemaphore);

	// returns pdFALSE if xBlockTime expired without the semaphore becoming available.
	if (xSemaphoreTake(xUARTSemaphore, mainUART_TIMEOUT) == pdFAIL)
	{
		OSRAMClear();
		OSRAMStringDraw("FAIL UART SEM", 0, 0);
		while (true)
			;
	}

	/* Create the queue used to pass message to vPrintTask. */
	xPrintQueue = xQueueCreate(mainQUEUE_SIZE, sizeof(char *));

	/* Create the queue used to pass the temperature value to vFilterTask */
	xSensorQueue = xQueueCreate(mainQUEUE_SIZE, sizeof(int));

	/* Start the standard demo tasks. */
	/* 	vStartIntegerMathTasks(tskIDLE_PRIORITY);
		vStartPolledQueueTasks(mainQUEUE_POLL_PRIORITY);
		vStartSemaphoreTasks(mainSEM_TEST_PRIORITY);
		vStartBlockingQueueTasks(mainBLOCK_Q_PRIORITY);
	 */
	/* Start the tasks defined within the file. */
	//	xTaskCreate( vCheckTask, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	xTaskCreate(vPrintTask, "Print", configMINIMAL_STACK_SIZE, NULL, mainDISPLAY_TASK_PRIORITY, NULL);
	xTaskCreate(vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainSENSOR_TASK_PRIORITY, NULL);
	xTaskCreate(vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainFILTER_TASK_PRIORITY, NULL);
	xTaskCreate(vUARTIntHandler, "UARTHandler", configMINIMAL_STACK_SIZE, NULL, mainUART_TASK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */
	OSRAMClear();
	OSRAMStringDraw("FAIL SCHEDULER START", 0, 0);

	return 0;
}
/*-----------------------------------------------------------*/

static void vCheckTask(void *pvParameters)
{
	portBASE_TYPE xErrorOccurred = pdFALSE;
	TickType_t xLastExecutionTime;
	const char *pcPassMessage = "PASS";
	const char *pcFailMessage = "FAIL";

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();

	for (;;)
	{
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil(&xLastExecutionTime, mainCHECK_DELAY);

		/* Has an error been found in any task? */

		if (xAreIntegerMathsTaskStillRunning() != pdTRUE)
		{
			xErrorOccurred = pdTRUE;
		}

		if (xArePollingQueuesStillRunning() != pdTRUE)
		{
			xErrorOccurred = pdTRUE;
		}

		if (xAreSemaphoreTasksStillRunning() != pdTRUE)
		{
			xErrorOccurred = pdTRUE;
		}

		if (xAreBlockingQueuesStillRunning() != pdTRUE)
		{
			xErrorOccurred = pdTRUE;
		}

		/* Send either a pass or fail message.  If an error is found it is
		never cleared again.  We do not write directly to the LCD, but instead
		queue a message for display by the print task. */
		if (xErrorOccurred == pdTRUE)
		{
			xQueueSend(xPrintQueue, &pcFailMessage, portMAX_DELAY);
		}
		else
		{
			xQueueSend(xPrintQueue, &pcPassMessage, portMAX_DELAY);
		}
	}
}

/*-----------------------------------------------------------*/

static void vSensorTask(void *pvParameters)
{
	TickType_t xLastExecutionTime;
	portBASE_TYPE xErrorOccurred = pdFALSE;

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();
	for (;;)
	{
		/* Perform this check every mainSENSOR_DELAY milliseconds. */
		vTaskDelayUntil(&xLastExecutionTime, mainSENSOR_DELAY);

		temperature = (temperature == 100) ? 0 : temperature + 1;

		if (xQueueSend(xSensorQueue, &temperature, 0) != pdPASS)
		{
			OSRAMClear();
			OSRAMStringDraw("QUEUE FULL", 0, 0);
			while (true)
				;
		} /*
		 if (xQueueOverwrite(xSensorQueue, temperature) != pdPASS)
		 {
			 OSRAMClear();
			 OSRAMStringDraw("FAIL FILTER", 0, 0);
			 while (true)
				 ;
		 } */
	}
}
/*-----------------------------------------------------------*/

static void vFilterTask(void *pvParameters)
{
	static uint8_t dataCounter = 0;
	TickType_t xLastExecutionTime = xTaskGetTickCount();
	int8_t startIndex; // variable to store the start index for the accumulator loop
	int8_t i;

	for (;;)
	{
		vTaskDelayUntil(&xLastExecutionTime, mainFILTER_TIMEOUT);

		/* if the queue is empty or the semaphore couldn't be taken */
		while (uxQueueMessagesWaiting(xSensorQueue) == 0 || xSemaphoreTake(xFilterSemaphore, mainFILTER_TIMEOUT) == pdFAIL)
			;

		int8_t sampleValue;
		if (xQueueReceive(xSensorQueue, &sampleValue, 0) == pdPASS)
		{
			int16_t accum = 0;

			// the first time we'll take the average on the amount of samples that we have or the last N samples
			if (dataCounter != MAX_N)
			{
				sampledData[dataCounter] = sampleValue;
				dataCounter++;
				startIndex = dataCounter - N;
				startIndex = startIndex <= 0 ? 0 : startIndex; /* if we have more samples than N, we take the average on the last N samples
				else if we have less than N samples, we take the average on all the samples we have
				*/
				for (i = startIndex; i < dataCounter; i++)
				{
					accum += sampledData[i];
				}
			}
			// after we fill the whole array we'll take the average on the last N samples after we shift the array
			else
			{
				for (i = 1; i < MAX_N; i++) // here we shift the array to the left (we discard the oldest value)
				{
					sampledData[i - 1] = sampledData[i];
				}

				sampledData[MAX_N - 1] = sampleValue;

				for (i = 0; i < MAX_N; i++) // here we take the average on the last N samples
				{
					accum += sampledData[i];
				}
			}

			int8_t average = accum / N;
			if (xQueueSend(xPrintQueue, &average, 0) != pdPASS)
			{
				OSRAMClear();
				OSRAMStringDraw("FILTER FAIL", 0, 0);
				while (true)
					;
			}
		}
		xSemaphoreGive(xFilterSemaphore);
	}
}

/*-----------------------------------------------------------*/
static void prvSetupHardware(void)
{
	/* Setup the PLL. */
	SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);

	/* Enable the UART.  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	/* Set GPIO A0 and A1 as peripheral function.  They are used to output the
	UART signals. */
	GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_DIR_MODE_HW);

	/* Configure the UART for 8-N-1 operation. */
	UARTConfigSet(UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE);

	/* Enable Rx interrupts. */
	HWREG(UART0_BASE + UART_O_IM) |= UART_INT_RX;
	IntPrioritySet(INT_UART0, configKERNEL_INTERRUPT_PRIORITY);
	IntEnable(INT_UART0);

	/* Initialise the LCD> */
	OSRAMInit(false);
	OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
	OSRAMStringDraw("LM3S811 demo", 16, 1);
}
/*-----------------------------------------------------------*/

void vUART_ISR(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	unsigned long ulStatus = UARTIntStatus(UART0_BASE, pdTRUE);
	UARTIntClear(UART0_BASE, ulStatus);

	xSemaphoreGiveFromISR(xUARTSemaphore, &xHigherPriorityTaskWoken);
}

void vUARTIntHandler(void)
{
	volatile char uartBuffer[UART_BUFFER_SIZE];
	volatile uint8_t uartBufferIndex = 0;

	for (;;)
	{

		while(xSemaphoreTake(xUARTSemaphore, portMAX_DELAY) == pdFALSE);

		while (UARTCharsAvail(UART0_BASE))
		{
			// Returns the character read from the specified port, cast as a long.
			long receivedChar = UARTCharGet(UART0_BASE);

			if (receivedChar == '\n' || receivedChar == '\r')
			{
				// Process the received N value
				uartBuffer[uartBufferIndex] = '\0'; // Null-terminate the buffer
				uint8_t newN = atoi(uartBuffer);	// Convert the buffer to an integer

				// Update the N value
				if (newN > 0 && newN <= MAX_N)
				{
					N = newN;
					xSemaphoreGive(xFilterSemaphore);
				}

				// Clear the buffer index
				uartBufferIndex = 0;
			}
			else if (uartBufferIndex < UART_BUFFER_SIZE - 1)
			{
				// Add the received character to the buffer
				uartBuffer[uartBufferIndex] = receivedChar;
				uartBufferIndex++;
			}

			OSRAMStringDraw("UART H", 0, 0);

		}
	}
}
/*-----------------------------------------------------------*/
/*
static void vUARTIntHandler(void)
{
	TickType_t xLastExecutionTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xLastExecutionTime, mainFILTER_TIMEOUT);

		while (xSemaphoreTake(xUARTSemaphore, mainUART_TIMEOUT) == pdFAIL) // xSemaphoreTake(xNFilterSemaphore, 5000) == pdFAIL ||
			;
		OSRAMStringDraw("UART H", 0, 0);

		char newN[UART_BUFFER_SIZE];
		for (uint8_t i = 0; i < UART_BUFFER_SIZE; i++)
		{
			long res = UARTCharNonBlockingGet(UART0_BASE);
			if (res == -1)
				break;
			else
				N = res - 48;
		}
	}
} */
/*
The UART interrupt handler is triggered whenever new data is received, allowing your program to receive and process data in real-time without stopping or blocking.*/

/*-----------------------------------------------------------*/

static void vPrintTask(void *pvParameters)
{
	float fValue;
	unsigned portBASE_TYPE uxLine = 0, uxRow = 0;
	TickType_t xLastExecutionTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xLastExecutionTime, mainFILTER_TIMEOUT);
		// OSRAMClear();

		/* Wait for a message to arrive. */
		if (xQueueReceive(xPrintQueue, &fValue, mainLCD_TIMEOUT) == pdTRUE)
		{
			OSRAMStringDraw("Print OK", 1, 1);
		}
		else
		{
			OSRAMStringDraw("Print FAIL", 1, 1);
		}
	}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */
	OSRAMClear();
	OSRAMStringDraw("OVERFLOW", 0, 0);
	for (;;)
		;
}