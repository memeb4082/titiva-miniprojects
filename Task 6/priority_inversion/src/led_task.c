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
 * highPriorityTask which handles processing the ISR result to adjust
 * the LED per the button inputs.
 *
 * highPriorityTask uses semaphore take to wait until it receives a
 * semaphore from the button ISR.  Once it does, then it processes the button
 * pressed.  Each time the SW1 or SW2 button is pressed, the LED index is
 * updated based on which button has been pressed and the corresponding LED
 * lights up.
 *
 * When either user switch SW1 or SW2 on the EK-TM4C1294XL is pressed, an
 * interrupt is generated and the switch pressed is logged in the global
 * variable g_pui32ButtonPressed.  Then the binary semaphore is given to
 * highPriorityTask before yielding to it.  This is an example of
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
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "drivers/rtos_hw_drivers.h"
/*-----------------------------------------------------------*/

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
extern SemaphoreHandle_t xSemaphore;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void highPriorityTask( void *pvParameters );
static void mediumPriorityTask( void *pvParameters );
static void lowPriorityTask( void *pvParameters );


/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vLEDTask( void );

/* 
 * Hardware configuration for the LEDs.
 */
static void prvConfigureLED( void );

/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */
static void prvConfigureButton( void );

/*
 * Shared resource to alter within tasks
 */
int resource = 0;

// system clock for UART
extern uint32_t g_ui32SysClock;


/*-----------------------------------------------------------*/

void vLEDTask( void )
{
    /* Light the initial LED. */
    prvConfigureLED();

 
    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Clear the terminal and print the welcome message.
    //
    UARTprintf("\nRunning priority inversion example\n");

    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name for the LED Task - for debug only as it is not used by
     *    the kernel.
     *  - The size of the stack to allocate to the task.
     *  - The parameter passed to the task - just to check the functionality.
     *  - The priority assigned to the task.
     *  - The task handle is not required, so NULL is passed. */
    xTaskCreate( highPriorityTask,
                 "high",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 3,
                 NULL );

    xTaskCreate( mediumPriorityTask,
                "medium",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL );
    
    xTaskCreate( lowPriorityTask,
                "low",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL );
}
/*-----------------------------------------------------------*/

static void highPriorityTask( void *pvParameters )
{
    int loopcount, t1count;
    loopcount = 0;
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

    for( ;; )
    {
        loopcount++;

        vTaskDelay(xDelay);

        UARTprintf("Running high priority task \n");
        /* Block until resource is available. */
        if( xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdPASS)
        {
            TickType_t start_of_crtical = xTaskGetTickCount();
            UARTprintf("High priority task in critical section \n");

            /* do work on locked resource */
            for (t1count = 0; t1count < 100000; t1count++) {
                    resource += 1;
                }

            /* unlock resource */
            UARTprintf("High priority task leaving the critical section \n");
            TickType_t end_of_crtical = xTaskGetTickCount();
            UARTprintf("High priority task run time: %d ms \n", (end_of_crtical - start_of_crtical) * portTICK_PERIOD_MS);
            xSemaphoreGive(xSemaphore);
        }

        //Calculate run time and print to terminal

    }
}

static void mediumPriorityTask( void *pvParameters )
{

    int icount1, icount2;
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

    for( ;; )
    {
        vTaskDelay(xDelay);

        UARTprintf("Running medium priority task \n");

        for(icount1 = 0; icount1 < 100000; icount1++) {
         	   for(icount2 = 0; icount2 < 100; icount2++);
        }
        

    }
}

static void lowPriorityTask( void *pvParameters )
{
    int t2count;
    
    UARTprintf("Running low priority task \n");

    for( ;; )
    {
        /* Lock the resource. */
        if( xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdPASS)
        {
            UARTprintf("Low priority task in critical section \n");

            /* do work on locked resource */
            for (t2count = 0; t2count < 400000; t2count++) {
                    resource += 1;
                }

            /* unlock resource */
            UARTprintf("Low priority task leaving the critical section \n");
            xSemaphoreGive(xSemaphore);
        }

    }
}


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, g_ui32SysClock);
}

/*-----------------------------------------------------------*/

static void prvConfigureLED( void )
{
    /* Configure initial LED state.  PinoutSet() has already configured
     * LED I/O. */
    LEDWrite(LED_D1, LED_D1);
}
/*-----------------------------------------------------------*/

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

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */

    /* Only the full demo uses the tick hook so there is no code is
        executed here. */
}

