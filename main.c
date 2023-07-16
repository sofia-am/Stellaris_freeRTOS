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

#include <string.h>

#define mainQUEUE_TIMEOUT 3000
#define mainFILTER_TIMEOUT 1000
#define mainLCD_TIMEOUT 1000
#define mainUART_TIMEOUT 3000

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY ((TickType_t)5000 / portTICK_PERIOD_MS)
#define mainTOP_DELAY ((TickType_t)2000/ portTICK_PERIOD_MS)

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
#define AXIS_START 30

/*
 * Configure the processor and peripherals for this demo.
 */
static void prvSetupHardware(void);

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

/**
 * @brief Map temperature to a 2 byte array.
 *
 * @param temp
 * @param graph
 */
void intToAscii(int num, char *buffer, int bufferSize);

/**
 * @brief Generate a random number between -5 and 5.
 *
 * @return int
 */
int customRand(void);

/**
 * @brief Convert an integer to an ASCII string.
 *
 * @param num
 * @param buffer
 * @param bufferSize
 */
void displayTemperatureGraph(int temperature, uint8_t graph[2]);

/**
 * @brief Print a string via UART.
 *
 * @param string
 */
void printViaUART(char *string);

/**
 * @brief Task that prints information about the CPU's usage like the top program.
 *
 * @param pvParameters
 */
void vTopLikeTask(char *pvParameters);

void vTaskGetStats(signed char *pcWriteBuffer);

char *longToChar(unsigned long value, char *ptr, int base);

static volatile char *pcNextChar;

/* Number of samples taken by the filter */
static uint8_t N = 1;

// VER GENERADOR DE RANDS
#define RAND_MAX 3
static int temperature = 18;
uint16_t accum;
uint8_t random = 0;

int8_t sampledData[MAX_N];

UBaseType_t uxHighWaterMarkSensor;
UBaseType_t uxHighWaterMarkFilter;
UBaseType_t uxHighWaterMarkPrint;
UBaseType_t uxHighWaterMarkUART;

/* The semaphore used to wake the button handler task from within the GPIO
interrupt handler. */
SemaphoreHandle_t xUARTSemaphore;

/* The semaphore used to change the value of the N parameter */
SemaphoreHandle_t xFilterSemaphore;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xPrintQueue;

/* The queue used to send temperature data to the filter. */
QueueHandle_t xSensorQueue;

static uint32_t g_seed = 0xDEADBEEF; // Initial seed value

/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the semaphore used to change variable N */
	vSemaphoreCreateBinary(xFilterSemaphore);
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
	xPrintQueue = xQueueCreate(mainQUEUE_SIZE, sizeof(uint8_t));

	/* Create the queue used to pass the temperature value to vFilterTask */
	xSensorQueue = xQueueCreate(mainQUEUE_SIZE, sizeof(int));

	/* Start the tasks defined within the file. */
	xTaskCreate(vPrintTask, "Print", configMINIMAL_STACK_SIZE, NULL, mainDISPLAY_TASK_PRIORITY, NULL);
	xTaskCreate(vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainSENSOR_TASK_PRIORITY, NULL);
	xTaskCreate(vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainFILTER_TASK_PRIORITY, NULL);
	xTaskCreate(vUARTIntHandler, "UART", configMINIMAL_STACK_SIZE, NULL, mainUART_TASK_PRIORITY, NULL);
	xTaskCreate(vTopLikeTask, "Top", configMINIMAL_STACK_SIZE, NULL, mainDISPLAY_TASK_PRIORITY - 1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */
	OSRAMClear();
	OSRAMStringDraw("FAIL SCHEDULER START", 0, 0);

	return 0;
}
/*-----------------------------------------------------------*/

static void vSensorTask(void *pvParameters)
{
	TickType_t xLastExecutionTime;
	int randomTemp = 0;
	portBASE_TYPE xErrorOccurred = pdFALSE;

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();
	for (;;)
	{
		/* Perform this check every mainSENSOR_DELAY milliseconds. */
		temperature = 20;
		vTaskDelayUntil(&xLastExecutionTime, mainSENSOR_DELAY);
		randomTemp = customRand();
		temperature += randomTemp;

		uxHighWaterMarkSensor = uxTaskGetStackHighWaterMark(NULL);

		if (xQueueSend(xSensorQueue, &temperature, mainCHECK_DELAY) != pdPASS)
		{
			OSRAMClear();
			OSRAMStringDraw("QUEUE FULL", 0, 0);
			while (true)
				;
		}
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
		if (xQueueReceive(xSensorQueue, &sampleValue, mainCHECK_DELAY) == pdTRUE)
		{
			accum = 0;

			// the first time we'll take the average on the amount of samples that we have or the last N samples
			if (dataCounter < MAX_N)
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

				for (i = MAX_N - N; i < MAX_N; i++) // here we take the average on the last N samples
				{
					accum += sampledData[i];
				}
			}

			int8_t average = accum / N;
			if (xQueueSend(xPrintQueue, &average, portMAX_DELAY) != pdPASS)
			{
				OSRAMClear();
				OSRAMStringDraw("FILTER FAIL", 0, 0);
				while (true)
					;
			}
		}

		uxHighWaterMarkFilter = uxTaskGetStackHighWaterMark(NULL);
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

	/* Configure the UART for 19200 - 8 bits - no parity - 1 bit stop operation. */
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
/*-----------------------------------------------------------*/

/* The UART interrupt handler is triggered whenever new data is received, allowing the program to receive and process data in real-time without stopping or blocking. */
void vUARTIntHandler(void)
{
	long receivedChar;
	uint8_t newN;

	for (;;)
	{

		while (xSemaphoreTake(xUARTSemaphore, portMAX_DELAY) == pdFALSE)
			;

		// Returns true if there is data in the receive FIFO or false if there is no data in the receive FIFO.
		while (UARTCharsAvail(UART0_BASE))
		{
			// Returns the character read from the specified port, cast as a long.
			receivedChar = UARTCharGet(UART0_BASE);

			if (receivedChar == '+')
			{
				// Update the N value
				newN = N + 1;
				if (newN <= MAX_N)
				{
					N = newN;
					xSemaphoreGive(xFilterSemaphore);
				}
			}
			else if (receivedChar == '-')
			{
				// Update the N value
				newN = N - 1;
				if (newN > 0)
				{
					N = newN;
					xSemaphoreGive(xFilterSemaphore);
				}
			}
		}

		uxHighWaterMarkUART = uxTaskGetStackHighWaterMark(NULL);
	}
}
/*-----------------------------------------------------------*/

static void vPrintTask(void *pvParameters)
{
	OSRAMClear();
	uint8_t averageValue;
	uint8_t byteTemp[2];
	char *N_ascii[5];

	int displayCounter = AXIS_START + 1;
	int endIndex = 96 - AXIS_START;

	TickType_t xLastExecutionTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xLastExecutionTime, mainFILTER_TIMEOUT);

		/* Wait for a message to arrive. */

		if (xQueueReceive(xPrintQueue, &averageValue, portMAX_DELAY) == pdTRUE)
		{
			// we get the two byte array to represent the temperature
			displayTemperatureGraph(averageValue, byteTemp);
			intToAscii(averageValue, N_ascii, 5);
			OSRAMStringDraw("t= ", 0, 0);
			OSRAMStringDraw(N_ascii, 0, 1);
			intToAscii(N, N_ascii, 5);
			OSRAMStringDraw("N=", 15, 0);
			OSRAMStringDraw(N_ascii, 15, 1);

			// Display the image on the OLED display
			unsigned char yaxis[] = {0xFF, 0xFF};
			OSRAMImageDraw(yaxis, AXIS_START, 0, 1, 2);

			uint8_t xaxis[] = {0x00, 0x80};
			OSRAMImageDraw(byteTemp, displayCounter, 0, 1, 2);
			for (int i = displayCounter + 1; i < 96; i++)
			{
				OSRAMImageDraw(xaxis, i, 0, 1, 2);
			}
			if (displayCounter < 96)
				displayCounter++;
			else
			{
				displayCounter = AXIS_START + 1;
			}
		}
		else
		{
			OSRAMStringDraw("Print FAIL", 1, 1);
		}

		uxHighWaterMarkPrint = uxTaskGetStackHighWaterMark(NULL);
	}
}
/*-----------------------------------------------------------*/

void vTopLikeTask(char *pvParameters)
{
	TickType_t xLastExecutionTime = xTaskGetTickCount();
	char *pcWriteBuffer;
	for (;;)
	{
		vTaskDelayUntil(&xLastExecutionTime, mainTOP_DELAY);
		vTaskGetStats(pcWriteBuffer);
	}
}

/*------------------------- Utilities --------------------------------*/

void vTaskGetStats(signed char *pcWriteBuffer)
{
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime, ulStatsAsPercentage;

	char *taskName;
	char *taskTime;
	char *taskPercentage;

	char buffer[10];
	uint32_t ulRunTimeCounter;

	/* Make sure the write buffer does not contain a string. */
	*pcWriteBuffer = 0x00;

	/* Take a snapshot of the number of tasks in case it changes while this
	function is executing. */
	uxArraySize = uxTaskGetNumberOfTasks();

	/* Allocate a TaskStatus_t structure for each task.  An array could be
	allocated statically at compile time. */
	pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

	if (pxTaskStatusArray != NULL)
	{
		printViaUART("\nTask		Time		Percentage	StackHighWatermark\n");
		printViaUART("******************************************************************\n");

		/* Generate raw status information about each task. */
		uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,
										   uxArraySize,
										   &ulTotalRunTime);

		/* For percentage calculations. */
		ulTotalRunTime /= 100UL;

		/* Avoid divide by zero errors. */
		if (ulTotalRunTime > 0)
		{
			/* For each populated position in the pxTaskStatusArray array,
			format the raw data as human readable ASCII data. */
			for (x = 0; x < uxArraySize; x++)
			{
				/* What percentage of the total run time has the task used?
				This will always be rounded down to the nearest integer.
				ulTotalRunTimeDiv100 has already been divided by 100. */
				ulStatsAsPercentage =
					pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

				if (ulStatsAsPercentage > 0UL)
				{
					taskName = pxTaskStatusArray[x].pcTaskName;
					ulRunTimeCounter = pxTaskStatusArray[x].ulRunTimeCounter;
					printViaUART(taskName);
					printViaUART("\t\t");
					printViaUART(longToChar(ulRunTimeCounter, buffer, 10));
					printViaUART("\t\t");
					printViaUART(longToChar(ulStatsAsPercentage, buffer, 10));
					printViaUART("%\t\t");
					printViaUART(longToChar(uxTaskGetStackHighWaterMark(pxTaskStatusArray[x].xHandle), buffer, 10));
					printViaUART(" words\r\n");	
				}
				else
				{
					/* If the percentage is zero here then the task has
					consumed less than 1% of the total run time. */
					taskName = pxTaskStatusArray[x].pcTaskName;
					ulRunTimeCounter = pxTaskStatusArray[x].ulRunTimeCounter;

					printViaUART(taskName);
					printViaUART("\t\t");
					taskTime = longToChar(ulRunTimeCounter, buffer, 10);
					printViaUART(taskTime);
					printViaUART("\t\t");
					printViaUART("<1%\t\t");
					printViaUART(longToChar(uxTaskGetStackHighWaterMark(pxTaskStatusArray[x].xHandle), buffer, 10));
					printViaUART(" words\r\n");
				}

				pcWriteBuffer += strlen((char *)pcWriteBuffer);
			}
		}		
	}
	else
	{
		printViaUART("Struct pxTaskStatusArray NULL\n");
	}
	/* The array is no longer needed, free the memory it consumes. */
		vPortFree(pxTaskStatusArray);
}

char *longToChar(unsigned long value, char *ptr, int base)
{

	unsigned long t = 0, res = 0;
	unsigned long tmp = value;
	int count = 0;

	if (NULL == ptr)
	{
		return NULL;
	}

	if (tmp == 0)
	{
		count++;
	}

	while (tmp > 0)
	{
		tmp = tmp / base;
		count++;
	}

	ptr += count;

	*ptr = '\0';

	do
	{

		res = value - base * (t = value / base);
		if (res < 10)
		{
			*--ptr = '0' + res;
		}
		else if ((res >= 10) && (res < 16))
		{
			*--ptr = 'A' - 10 + res;
		}

	} while ((value = t) != 0);

	return (ptr);
}

int customRand(void)
{
	g_seed ^= g_seed << 13;
	g_seed ^= g_seed >> 17;
	g_seed ^= g_seed << 5;

	int32_t random_number = (int32_t)(g_seed % 11) - 5; // Scale and shift the range

	return random_number;
}

void intToAscii(int num, char *buffer, int bufferSize)
{
	if (num == 0)
	{
		buffer[0] = '0';
		buffer[1] = '\0';
		return;
	}

	int i = 0;
	// Build the ASCII string in reverse order
	while (num > 0 && i < bufferSize - 1)
	{
		int digit = num % 10;
		buffer[i] = '0' + digit;
		num /= 10;
		i++;
	}

	// Add the null terminator
	buffer[i] = '\0';

	// Reverse the string
	int length = i;
	for (int j = 0; j < length / 2; j++)
	{
		char temp = buffer[j];
		buffer[j] = buffer[length - j - 1];
		buffer[length - j - 1] = temp;
	}
}

void displayTemperatureGraph(int temp, uint8_t graph[2])
{
	if (temp > 23)
	{
		graph[0] = 0x01;
		graph[1] = 0x80;
		return;
	}
	else if (temp < 10)
	{
		graph[0] = 0x00;
		graph[1] = 0x80;
		return;
	}

	int pixel = 7 - (temp % 8);
	// Calculated so the value 10 (lowest temp possible) is mapped to pixel 6 of the lowest row.

	uint8_t lowerRow = 0x80; // We set the bit 7 to 1 to graph the x axis.
	uint8_t upperRow = 0;

	if (temp < 16)
	{
		lowerRow |= (1 << pixel);
	}
	else
	{
		upperRow |= (1 << pixel);
	}

	graph[0] = upperRow;
	graph[1] = lowerRow;
}

void printViaUART(char *string)
{
	while (*string != '\0')
	{
		UARTCharPut(UART0_BASE, *string);
		string++;
	}
}