/*
 * led_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/******************************************************************************
 *
 * vLEDTask turns on the initial LED, configurations the buttons, and creates
 * prvProcessSwitchInputTask which handles processing the ISR result to adjust
 * the LED per the button inputs.
 *
 * prvProcessSwitchInputTask uses semaphore take to wait until it receives a
 * semaphore from the button ISR.  Once it does, then it processes the button
 * pressed.  Each time the SW1 or SW2 button is pressed, the LED index is
 * updated based on which button has been pressed and the corresponding LED
 * lights up.
 *
 * When either user switch SW1 or SW2 on the EK-TM4C1294XL is pressed, an
 * interrupt is generated and the switch pressed is logged in the global
 * variable g_pui32ButtonPressed.  Then the binary semaphore is given to
 * prvProcessSwitchInputTask before yielding to it.  This is an example of
 * using binary semaphores to defer ISR processing to a task.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "drivers/i2cOptDriver.h"
#include "driverlib/i2c.h"
#include "drivers/opt3001.h"
/*-----------------------------------------------------------*/
#define QUEUE_LENGTH 4
/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

/*
 * The binary semaphore used by the switch ISR & task.
 */
extern SemaphoreHandle_t xButton1Semaphore;
extern SemaphoreHandle_t xButton2Semaphore;
extern SemaphoreHandle_t xIC2MasterSemaphore;
extern SemaphoreHandle_t xOPTSemaphore;
QueueHandle_t xOptQueue = NULL;

extern uint32_t g_ui32SysClock;

bool button = false;

typedef struct {
    float filteredLux;
    float* lux;
    uint32_t timestamp;
} luxData_t;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvLSTask( void *pvParameters );
static void prvQueueReceiveTask( void *pvParameters );
/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */
static void prvConfigureButton( void );

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vCreateLightSensorTask( void );

/* Handles the timer interrupt and signals when the read/write task is completed */
void xTimerHandler( void );

/*-----------------------------------------------------------*/

void vCreateLightSensorTask( void )
{
    /* Configure the button to generate interrupts. */
    prvConfigureButton();
    /* Create the queue used to send data between tasks. */
    xOptQueue = xQueueCreate( QUEUE_LENGTH, sizeof( luxData_t ) );
    if ( xOptQueue == NULL )
    {
        UARTprintf("Queue Creation Failed\n");
        return;
    }

    xTaskCreate( prvLSTask,
                 "Light Sensor",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 10,
                 NULL );
    xTaskCreate( prvQueueReceiveTask,
                    "Queue Receive",
                    configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 10,
                    NULL );
}
/*-----------------------------------------------------------*/


static void prvLSTask( void *pvParameters ) {
    // UARTprintf("Light Sensor Task Started\n");
    // Wait for sensor to power up (important!)

    // Now initialize the OPT3001 sensor
    sensorOpt3001Init();

    bool success;
    uint16_t rawData = 0;
    float convertedLux = 0;

    // Test that sensor is set up correctly
    // UARTprintf("Testing OPT3001 Sensor:\n");
    success = sensorOpt3001Test();

    //stay here until sensor is working
    while (!success) {
        vTaskDelay(pdMS_TO_TICKS(100));  // Cooperative delay
        UARTprintf("Test Failed, Trying again\n");
        success = sensorOpt3001Test();
    }

    // UARTprintf("All Tests Passed!\n\n");
    bool button = false;

    // Loop Forever
    luxData_t send;
    float average = 0;
    float buffer[QUEUE_LENGTH];
    int i = 0;
    while(1)
    {
        // Check if the button was pressed
        if (xSemaphoreTake(xButton1Semaphore, 50) == pdTRUE)
        {
            button = !button;
        }
        // Check if the light levels changed above/below thresholds
        // if (xSemaphoreTake(xOPTSemaphore, 50) == pdTRUE)
        // {
            // success = sensorOpt3001Read(&rawData);
            // if (success)
            // {
            //     sensorOpt3001Convert(rawData, &convertedLux);
            //     // if (convertedLux < 40.95) 
            //     //     UARTprintf("Low Light Event: %d Lux\n", (int)convertedLux);
            //     // else if (convertedLux > 2620.80)
            //     //     UARTprintf("High Light Event: %d Lux\n", (int)convertedLux);
            //     // else
            //     //     UARTprintf("Didnt Register Lux: %d\n", (int)convertedLux);
            //     UARTprintf("Lux: %d\n", (int)convertedLux);
            // }
        // }

        if (button)
        {
            if (i == QUEUE_LENGTH)
            {
                // Reset the counter and send struct to queue
                send.filteredLux = average / QUEUE_LENGTH;
                send.lux = buffer;
                send.timestamp = xTaskGetTickCount();
                xQueueSend(xOptQueue, &send, 0);
                average = 0;
                i = 0;
            }

            success = sensorOpt3001Read(&rawData);
            if (success)
            {
                sensorOpt3001Convert(rawData, &convertedLux);
                average += convertedLux;
                buffer[i] = convertedLux;
                i++;
            }
        }
    }
}

static void prvQueueReceiveTask( void *pvParameters )
{
    luxData_t received;
    for (;;)
    {
        if (xOptQueue != NULL)
        {
            // Wait for the queue to be filled
            if (xQueueReceive(xOptQueue, &received, portMAX_DELAY) == pdTRUE)
            {
                // Print the lux values received from the queue
                for (int i = 0; i < QUEUE_LENGTH; i++)
                {
                    UARTprintf("%d,%d\n", (int)received.lux[i], (int)received.filteredLux);
                }
                                                            //                 // Print the lux values received from the queue
                // for (int i = 0; i < QUEUE_LENGTH; i++)
                // {
                //     UARTprintf("%d, ", (int)received.lux[i]);
                // }
                // UARTprintf("Filt: %d\n", (int)received.filteredLux);
            }
        }
        else
        {
            // UARTprintf("Queue is NULL\n");
        }
        
    }
}

static void prvConfigureButton( void )
{
    /* Initialize the LaunchPad Buttons. */
    ButtonsInit();

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port F interrupt in the NVIC. */
    IntEnable(INT_GPIOJ);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}
/*-----------------------------------------------------------*/


void xButtonsHandler( void )
{
    BaseType_t xLEDTaskWoken;

    uint32_t ui32Status;

    /* Initialize the xLEDTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xLEDTaskWoken = pdFALSE;

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    /* Debounce the input with 200ms filter */
    if ((xTaskGetTickCount() - g_ui32TimeStamp ) > 150)
    {
        /* Log which button was pressed to trigger the ISR. */
        if ((ui32Status & USR_SW1) == USR_SW1)
        {
            g_pui32ButtonPressed = USR_SW1;
            /* Give the semaphore to unblock prvProcessSwitchInputTask.  */
            xSemaphoreGiveFromISR( xButton1Semaphore, &xLEDTaskWoken );
        }

        /* This FreeRTOS API call will handle the context switch if it is
         * required or have no effect if that is not needed. */
        portYIELD_FROM_ISR( xLEDTaskWoken );

    }

    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}
/*-----------------------------------------------------------*/

void xOptHandler( void ) {
    BaseType_t xOPTTaskWoken = pdFALSE;


    /* Read the PORT P interrupt status to find the cause of the interrupt. */
    uint32_t ui32Status = GPIOIntStatus(GPIO_PORTP_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(GPIO_PORTP_BASE, ui32Status);
    //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0);
    xSemaphoreGiveFromISR(xOPTSemaphore, &xOPTTaskWoken);
    portYIELD_FROM_ISR(xOPTTaskWoken);
}


void xI2CHandler(void)
{
    BaseType_t xSignalTaskWoken = pdFALSE;

    // Clear interrupt
    I2CMasterIntClear(I2C0_BASE);


    // Only give the semaphore when the I2C bus is idle (transfer finished)
    if (!I2CMasterBusy(I2C0_BASE))
    {
        xSemaphoreGiveFromISR(xIC2MasterSemaphore, &xSignalTaskWoken);
        portYIELD_FROM_ISR(xSignalTaskWoken);
    }
}