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
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "drivers/rtos_hw_drivers.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"

/* Display includes. */
#include "grlib.h"
#include "widget.h"
#include "canvas.h"

#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

#define MAX_FRUITS 2
#define FRUIT_WIDTH 10
#define FRUIT_HEIGHT 10
#define FRUIT_SPEED 9
#define BASKET_WIDTH 40
#define BASKET_HEIGHT 40
#define BASKET_SPEED 30

/*-----------------------------------------------------------*/
/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

extern volatile uint32_t g_ui32SysClock;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

/*
 * The binary semaphore used by the switch ISR & task.
 */
extern SemaphoreHandle_t xButtonSemaphore, xDisplaySemaphore, xLogicSemaphore, xVariableSemaphore;

/*
 * data structures for the graphics library
 */
tContext sContext;
tRectangle sRect, sRect2, sBasket, SFruit;

bool fruit_spawn = false, special = false, game_over = false, initial_press = false, start_menu = true, fruit_update = false;
int lives = 3, points = 0, enlarged = 1;

typedef struct
{
    bool active;
    bool special;
    int16_t x;
    int16_t y;
} Fruit;

Fruit fruits[MAX_FRUITS];

uint16_t x_pos_basket = 160;

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvDisplayTask(void *pvParameters);

static void prvLogicTask(void *pvParameters);

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vCreateTask(void);

/*
 * Hardware configuration for the LEDs.
 */
static void prvConfigureLED(void);

/*
 * Timer configuration
 */
static void prvConfigureHWTimer(void);

/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */
static void prvConfigureButton(void);
/*-----------------------------------------------------------*/

void vCreateDisplayTask(void)
{
    /* Light the initial LED. */
    prvConfigureLED();

    /* Configure the button to generate interrupts. */
    prvConfigureButton();

    /* Configure the hardware timer to run in periodic mode. */
    prvConfigureHWTimer();

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
    xTaskCreate(prvDisplayTask,
                "LED",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
}

void vCreateLogicTask(void)
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
    xTaskCreate(prvLogicTask,
                "MainTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
}

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

static void prvConfigureHWTimer(void)
{
    /* The Timer 0 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure Timer 0 in full-width periodic mode. */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 0A load value to run at 5 Hz. */
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 5); // 0.2 seconds

    /* Configure the Timer 0A interrupt for timeout. */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the Timer 0A interrupt in the NVIC. */
    IntEnable(INT_TIMER0A);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();

    //
    // Start the timer used in this example Task
    // You may need change where this timer is enabled
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
}

/*-----------------------------------------------------------*/
int timer_counter = 0;
int speed_increase = 0;
int timer_large = 0;
void xTimerHandler(void)
{

    BaseType_t xLEDTaskWoken;

    /* Initialize the xLEDTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xLEDTaskWoken = pdFALSE;
    timer_counter++;

    if (enlarged > 1)
    {
        timer_large++;
    }
    else
    {
        timer_large = 0;
    }

    if (timer_large > 60)
    {
        enlarged = 1;
    }

    if ((timer_counter % 10) == 0)
    {
        fruit_spawn = true;
    }
    if ((timer_counter % 70) == 0)
    {
        special = true;
    }
    if ((timer_counter % 50) == 0)
    {
        speed_increase++;
    }
    fruit_update = true;
    //     xSemaphoreGiveFromISR(xVariableSemaphore, &xLEDTaskWoken);
    // }
    /* Clear the hardware interrupt flag for Timer 0A. */
    xSemaphoreGiveFromISR(xLogicSemaphore, &xLEDTaskWoken);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Update only time based game variables here*/
    // e.g. fruit location
    // don't do full game logic here
}

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

    /* Debounce the input with 100ms filter */
    // Can reduce this value to increase response time of button
    // but if too small can lead to debouncing issues
    if ((xTaskGetTickCount() - g_ui32TimeStamp) > 100)
    {
        /* Log which button was pressed to trigger the ISR. */

        if ((ui32Status & USR_SW1) == USR_SW1)
        {
            x_pos_basket += BASKET_SPEED;
            g_pui32ButtonPressed = USR_SW1;
        }
        else if ((ui32Status & USR_SW2) == USR_SW2)
        {
            x_pos_basket -= BASKET_SPEED;
            g_pui32ButtonPressed = USR_SW2;
        }

        /* Give the semaphore to unblock prvProcessSwitchInputTask.  */
        xSemaphoreGiveFromISR(xButtonSemaphore, &xLEDTaskWoken);
        xSemaphoreGiveFromISR(xLogicSemaphore, &xLEDTaskWoken);
        initial_press = true;

        /* This FreeRTOS API call will handle the context switch if it is
         * required or have no effect if that is not needed. */
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }

    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}
/*-----------------------------------------------------------*/

/*--------------------------TASK FUNCTIONS ---------------------------------*/
// You could either add a second game logic task function and setup in this file
// or create a second src (.c) file and define a game logic task function there

void clear_screen(void)
{
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect2);
    return;
}

static void prvDisplayTask(void *pvParameters)
{
    /* Initialise the screen */

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    // able to clear the screen
    sRect2.i16XMin = 0;
    sRect2.i16YMin = 24;
    sRect2.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect2.i16YMax = 300;

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "Falling Fruit Example", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 8, 0);

    // displays the rules of the game outside the loop as it will be removed as soon as the game starts
    GrStringDrawCentered(&sContext, "Falling Fruit Game", -1, 160, 40, false);
    GrStringDrawCentered(&sContext, "Catch the fruit with the basket!", -1, 160, 60, false);
    GrStringDrawCentered(&sContext, "Move with buttons.", -1, 160, 80, false);
    GrStringDrawCentered(&sContext, "Each catch = +1 point", -1, 160, 100, false);
    GrStringDrawCentered(&sContext, "Miss 3 = Game Over!", -1, 160, 120, false);

    //
    // Main loop for display task
    //
    // temp var
    bool temp_game_over = game_over;
    bool temp_initial_press = initial_press;
    bool temp_start_menu = start_menu;
    for (;;)
    {
        /* Block until the Push Button ISR gives the semaphore. */
        if (xSemaphoreTake(xDisplaySemaphore, portMAX_DELAY) == pdPASS)
        {
            if (xSemaphoreTake(xVariableSemaphore, portMAX_DELAY) == pdPASS)
            {
                temp_game_over = game_over;
                temp_initial_press = initial_press;
                temp_start_menu = start_menu;
                xSemaphoreGive(xVariableSemaphore);
            }

            // if the first button is pressed and the start menu is still on clear it
            if (temp_game_over)
            {
                // game over
                clear_screen();

                char points_string[20];
                char lives_string[20];

                if (xSemaphoreTake(xVariableSemaphore, portMAX_DELAY) == pdPASS)
                {
                    usprintf(points_string, "  Score: %d  ", points);
                    usprintf(lives_string, "  Lives: %d  ", lives);
                    xSemaphoreGive(xVariableSemaphore);
                }

                //
                // Fill the top 24 rows of the screen with blue to create the banner.
                //
                sRect.i16XMin = 0;
                sRect.i16YMin = 0;
                sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
                sRect.i16YMax = 23;
                GrContextForegroundSet(&sContext, ClrDarkBlue);
                GrRectFill(&sContext, &sRect);
                GrContextFontSet(&sContext, &g_sFontCm20);

                //
                // Put a white box around the banner.
                //
                GrContextForegroundSet(&sContext, ClrWhite);
                GrRectDraw(&sContext, &sRect);

                /*
                Update the display string based on which LED is on
                */
                GrContextFontSet(&sContext, &g_sFontCm20);
                GrStringDrawCentered(&sContext, points_string, -1, GrContextDpyWidthGet(&sContext) / 4 * 3, 8, 0);
                GrStringDrawCentered(&sContext, lives_string, -1, GrContextDpyWidthGet(&sContext) / 4, 8, 0);
                GrStringDrawCentered(&sContext, "Game Over!", -1, 160, 100, false);
                for (;;)
                    ;
            }
            else if (temp_start_menu & temp_initial_press)
            {
                GrContextForegroundSet(&sContext, ClrBlack);
                GrRectFill(&sContext, &sRect2);
                if (xSemaphoreTake(xVariableSemaphore, portMAX_DELAY) == pdPASS)
                {
                    start_menu = false;
                    xSemaphoreGive(xVariableSemaphore);
                }
            }
            else if ((!temp_start_menu))
            {
                clear_screen();
                if (xSemaphoreTake(xVariableSemaphore, portMAX_DELAY) == pdPASS)
                {
                    taskENTER_CRITICAL();
                    GrContextForegroundSet(&sContext, ClrRed);
                    sBasket.i16XMin = x_pos_basket + BASKET_WIDTH * enlarged;
                    sBasket.i16YMin = 240 - BASKET_HEIGHT * enlarged;
                    sBasket.i16XMax = x_pos_basket - BASKET_WIDTH * enlarged;
                    sBasket.i16YMax = GrContextDpyHeightGet(&sContext) - 1;
                    GrRectFill(&sContext, &sBasket);
                    GrContextForegroundSet(&sContext, ClrBlack);
                    xSemaphoreGive(xVariableSemaphore);
                    taskEXIT_CRITICAL();
                }

                char points_string[20];
                char lives_string[20];

                // draws all of the fruits
                if (xSemaphoreTake(xVariableSemaphore, portMAX_DELAY) == pdPASS)
                {
                    for (int i = 0; i < MAX_FRUITS; i++)
                    {
                        if (fruits[i].active)
                        {
                            if (fruits[i].special == true)
                            {
                                GrContextForegroundSet(&sContext, ClrCyan);
                            }
                            else
                            {

                                GrContextForegroundSet(&sContext, ClrYellow);
                            }
                            GrRectFill(&sContext, &(tRectangle){
                                                      .i16XMin = fruits[i].x,
                                                      .i16YMin = fruits[i].y,
                                                      .i16XMax = fruits[i].x + FRUIT_WIDTH,
                                                      .i16YMax = fruits[i].y + FRUIT_HEIGHT});
                        }
                    }
                    xSemaphoreGive(xVariableSemaphore);
                }

                //
                // Fill the top 24 rows of the screen with blue to create the banner.
                //
                sRect.i16XMin = 0;
                sRect.i16YMin = 0;
                sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
                sRect.i16YMax = 23;
                GrContextForegroundSet(&sContext, ClrDarkBlue);
                GrRectFill(&sContext, &sRect);
                GrContextFontSet(&sContext, &g_sFontCm20);

                if (xSemaphoreTake(xVariableSemaphore, portMAX_DELAY) == pdPASS)
                {
                    usprintf(points_string, "  Score: %d  ", points);
                    usprintf(lives_string, "  Lives: %d  ", lives);
                    xSemaphoreGive(xVariableSemaphore);
                }

                //
                // Put a white box around the banner.
                //
                GrContextForegroundSet(&sContext, ClrWhite);
                GrRectDraw(&sContext, &sRect);

                /*
                Update the display string based on which LED is on
                */
                GrContextFontSet(&sContext, &g_sFontCm20);
                GrStringDrawCentered(&sContext, points_string, -1, GrContextDpyWidthGet(&sContext) / 4 * 3, 8, 0);
                GrStringDrawCentered(&sContext, lives_string, -1, GrContextDpyWidthGet(&sContext) / 4, 8, 0);
            }
        }
    }
}
static void prvLogicTask(void *pvParameters)
{
    for (;;)
    {
        // only runs if the timer indicates and update has occured
        if (xSemaphoreTake(xLogicSemaphore, portMAX_DELAY) == pdPASS)
        {
            taskENTER_CRITICAL();
            // takes the semaphore to make sure nothing else uses the semaphore
            if (xSemaphoreTake(xVariableSemaphore, portMAX_DELAY) == pdPASS)
            {
                if ((!start_menu) & (!game_over))
                {
                    // If the timer has set a fruit to be ready
                    if (fruit_spawn)
                    {
                        fruit_spawn = false;
                        for (int i = 0; i < MAX_FRUITS; i++)
                        {
                            if (!fruits[i].active)
                            {
                                fruits[i].x = rand() % ((GrContextDpyWidthGet(&sContext) - 1) - FRUIT_WIDTH);
                                fruits[i].y = 24;
                                fruits[i].active = true;
                                if (special)
                                {
                                    fruits[i].special = true;
                                    special = false;
                                }
                                else
                                {
                                    fruits[i].special = false;
                                }
                                break; // once it spawns one it stops
                            }
                        }
                    }
                    if (fruit_update)
                    {
                        for (int i = 0; i < MAX_FRUITS; i++)
                        {
                            if (fruits[i].active)
                            {
                                fruits[i].y += (FRUIT_SPEED + speed_increase);
                                if (fruits[i].y > (GrContextDpyHeightGet(&sContext) - 1)) // 240 pixels
                                {
                                    fruits[i].active = false;
                                    lives -= 1;
                                }
                            }
                        }
                    }
                    for (int i = 0; i < MAX_FRUITS; i++)
                    {
                        if (fruits[i].active)
                        {
                            if ((fruits[i].y > (240 - BASKET_HEIGHT * enlarged)) & (fruits[i].x > (x_pos_basket - BASKET_WIDTH * enlarged)) & (fruits[i].x < (x_pos_basket + BASKET_WIDTH * enlarged)))
                            {
                                fruits[i].active = false;
                                if (fruits[i].special == true)
                                {
                                    enlarged = 2;
                                }
                                points++;
                            }
                        }
                    }
                    if (lives <= 0)
                    {
                        game_over = true;
                    }
                }
                xSemaphoreGive(xVariableSemaphore);
                taskEXIT_CRITICAL();
            }
            xSemaphoreGive(xDisplaySemaphore);
        }
    }
}

/*-----------------------------------------------------------*/
