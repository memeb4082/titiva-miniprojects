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
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

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
extern SemaphoreHandle_t xButtonSemaphore, xButtonSemaphore2, semaphore_protect;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvLEDTask(void *pvParameters);
static void prvLEDTask2(void *pvParameters);
static void prvOkTask(void *pvParameters);

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vCreateLEDTask(void);
void vCreateLEDTask2(void);
void vCreateOkTask(void);

/*
 * Hardware configuration for the LEDs.
 */
static void prvConfigureLED(void);
static void prvConfigureUART(void);

/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */
static void prvConfigureButton(void);
/*-----------------------------------------------------------*/

void vCreateLEDTask(void)
{
    /* Light the initial LED. */
    prvConfigureLED();

    /* Configure the button to generate interrupts. */
    prvConfigureButton();

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
    xTaskCreate(prvLEDTask,
                "LED",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
}
void vCreateLEDTask2(void)
{
    /* Light the initial LED. */
    prvConfigureLED();

    /* Configure the button to generate interrupts. */
    prvConfigureButton();

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
    xTaskCreate(prvLEDTask2,
                "LED2",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
}
/*-----------------------------------------------------------*/

void vCreateOkTask(void)
{

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
    xTaskCreate(prvOkTask,
                "OKTASK",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}
/*-----------------------------------------------------------*/

uint8_t ui8LEDIndex = 0;

static void prvLEDTask(void *pvParameters)
{
    // gies 0->3
    for (;;)
    {
        /* Block until the ISR gives the semaphore. */
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdPASS)
        {

            if (xSemaphoreTake(semaphore_protect, portMAX_DELAY) == pdPASS)
            {
                /* If the right button is hit, either increment by 1 or reset the
                 * index to 0 if it is at 3. */
                IntMasterDisable();
                if (ui8LEDIndex == 3)
                {
                    ui8LEDIndex = 0;
                }
                else
                {
                    ui8LEDIndex++;
                }

                /* If the left button is hit, either decrement by 1 or reset the
                 * index to 3 if it is at 0. */

                /* Turn off all LED's. */
                LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0);

                /* Set the LED based on the updated Index. */
                switch (ui8LEDIndex)
                {
                case 0:
                    LEDWrite(LED_D1, LED_D1);
                    break;
                case 1:
                    LEDWrite(LED_D2, LED_D2);
                    break;
                case 2:
                    LEDWrite(LED_D3, LED_D3);
                    break;
                case 3:
                    LEDWrite(LED_D4, LED_D4);
                    break;
                }
                IntMasterEnable();
            }
            xSemaphoreGive(semaphore_protect); // give the semaphore once it is safe
        }
    }
}
/*-----------------------------------------------------------*/

static void prvLEDTask2(void *pvParameters)
{

    for (;;)
    {
        /* Block until the ISR gives the semaphore. */
        if (xSemaphoreTake(xButtonSemaphore2, portMAX_DELAY) == pdPASS)
        {

            if (xSemaphoreTake(semaphore_protect, portMAX_DELAY) == pdPASS)
            {
                IntMasterDisable();
                /* If the right button is hit, either increment by 1 or reset the
                 * index to 0 if it is at 3. */

                /* If the left button is hit, either decrement by 1 or reset the
                 * index to 3 if it is at 0. */

                if (ui8LEDIndex == 0)
                {
                    ui8LEDIndex = 3;
                }
                else
                {
                    ui8LEDIndex--;
                }
                /* Turn off all LED's. */

                LEDWrite(LED_D1 | LED_D2 | LED_D3 | LED_D4, 0);

                /* Set the LED based on the updated Index. */
                switch (ui8LEDIndex)
                {
                case 0:
                    LEDWrite(LED_D1, LED_D1);
                    break;
                case 1:
                    LEDWrite(LED_D2, LED_D2);
                    break;
                case 2:
                    LEDWrite(LED_D3, LED_D3);
                    break;
                case 3:
                    LEDWrite(LED_D4, LED_D4);
                    break;
                }
                IntMasterEnable();
            }
            xSemaphoreGive(semaphore_protect); // give the semaphore once it is safe
        }
    }
}
/*-----------------------------------------------------------*/
static void prvOkTask(void *pvParameters)
{
    prvConfigureUART();
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        UARTprintf("System OK\n");
    }
}

/*-----------------------------------------------------------*/

static void prvConfigureLED(void)
{
    /* Configure initial LED state.  PinoutSet() has already configured
     * LED I/O. */
    LEDWrite(LED_D1, LED_D1);
}
/*-----------------------------------------------------------*/

static void prvConfigureButton(void)
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

void xButtonsHandler(void)
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
    if ((xTaskGetTickCount() - g_ui32TimeStamp) > 100)
    {
        /* Log which button was pressed to trigger the ISR. */
        if ((ui32Status & USR_SW1) == USR_SW1)
        {
            g_pui32ButtonPressed = USR_SW1;
            xSemaphoreGiveFromISR(xButtonSemaphore2, &xLEDTaskWoken);
        }
        else if ((ui32Status & USR_SW2) == USR_SW2)
        {
            g_pui32ButtonPressed = USR_SW2;
            xSemaphoreGiveFromISR(xButtonSemaphore, &xLEDTaskWoken);
        }

        /* Give the semaphore to unblock prvProcessSwitchInputTask.  */

        /* This FreeRTOS API call will handle the context switch if it is
         * required or have no effect if that is not needed. */
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }

    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}
/*-----------------------------------------------------------*/
static void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 9600, 16000000);
}