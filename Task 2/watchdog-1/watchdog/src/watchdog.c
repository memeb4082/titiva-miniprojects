//*****************************************************************************
//
// watchdog.c - Watchdog timer example.
//
// Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/watchdog.h"
#include "driverlib/timer.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"

#define watchdog_counter_comp 2
#define reset_button_timer 8

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Watchdog (watchdog)</h1>
//!
//! This example application demonstrates the use of the watchdog as a simple
//! heartbeat for the system.  If the watchdog is not periodically fed, it will
//! reset the system.  Each time the watchdog is fed, the LED is inverted so
//! that it is easy to see that it is being fed, which occurs once every
//! second.  To stop the watchdog being fed and, hence, cause a system reset,
//! press the SW1 button.
//
//*****************************************************************************

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, g_ui32SysClock);
}

//****************************************************************************** */
// Adds a counter for the watchdog timer
//****************************************************************************** */
volatile int counter_watchdog_inv = 0;

//*****************************************************************************
//
// Flag to tell the watchdog interrupt handler whether or not to clear the
// interrupt (feed the watchdog).
//
//*****************************************************************************
volatile bool g_bFeedWatchdog = true;

//*****************************************************************************
//
// The interrupt handler for the watchdog.  This feeds the dog (so that the
// processor does not get reset) and blinks the LED connected to GPIO B3.
//
//*****************************************************************************
void WatchdogIntHandler(void)
{
    //
    // If we have been told to stop feeding the watchdog, return immediately
    // without clearing the interrupt.  This will cause the system to reset
    // next time the watchdog interrupt fires.
    //
    counter_watchdog_inv++;
    
    if (!g_bFeedWatchdog)
    {
        //It requires to return twice, because if it returns without being cleared it will wait until the interupt is triggered again, and at that point if the watchdog is not cleared and the interupt happens again it resets the system
        return;
    }
    else if ((counter_watchdog_inv > watchdog_counter_comp))
    {
        UARTprintf("Timer_runout resetting\n");
        g_bFeedWatchdog = false;
        // SysCtlDelay(g_ui32SysClock); // Adds a delay so the uart message doesnt die
        return;
    }
    else
    {
        //
        // Clear the watchdog interrupt.
        //
        UARTprintf("watchdog_timer: %d\n",counter_watchdog_inv);
        WatchdogIntClear(WATCHDOG0_BASE);
    }

    //
    // Invert the GPIO PN0 value.
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,
                 (GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0) ^
                  GPIO_PIN_0));
}

volatile int timer_counter = 0;
//**************************************************** */
// timer hand;er
void TimerHandler(void)
{

    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterDisable();

    timer_counter += 1;

    UARTprintf("timer: %d\n", timer_counter);

    if ((timer_counter >= reset_button_timer) & g_bFeedWatchdog)
    {
        UARTprintf("Too Late to feed - Resetting shortly\n");
        // SysCtlDelay(g_ui32SysClock); //lets the message get out
        g_bFeedWatchdog = false;
        WatchdogLock(WATCHDOG0_BASE);
    }

    IntMasterEnable();
}

//*****************************************************************************
//
// This function is called when the SW1 button is pressed.
//
//*****************************************************************************
static int32_t
SW1ButtonPressed(void)
{
    //
    // Set the flag that tells the interrupt handler not to clear the
    // watchdog interrupt.
    //
    counter_watchdog_inv = 0;
    timer_counter = 0;
    // feed watchdog
    UARTprintf("Reseting watchdog counter\n");
    return (0);
}

//
void timer_setup(void)
{
    TimerDisable(TIMER0_BASE, TIMER_A); // Ensure the timer is off before setup
    //
    // Configure the two 32-bit periodic timers.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock); // one second timer

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void watchdog_setup(void)
{
    // unlock the watchdog
    WatchdogUnlock(WATCHDOG0_BASE);

    WatchdogResetDisable(WATCHDOG0_BASE);

    //
    // Enable the watchdog interrupt.
    //
    IntEnable(INT_WATCHDOG);

    //
    // Set the period of the watchdog timer to 5 second.
    //
    WatchdogReloadSet(WATCHDOG0_BASE, 5 * g_ui32SysClock); // I know it could be better but it reminds me how it was done originally

    //
    // Enable reset generation from the watchdog timer.
    //
    WatchdogResetEnable(WATCHDOG0_BASE);
}
//*****************************************************************************
//
// This example demonstrates the use of the watchdog timer.
//
//*****************************************************************************
int main(void)
{
    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                         SYSCTL_OSC_MAIN |
                                         SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_240),
                                        120000000);

    //
    // Initialize the buttons driver.
    //
    ButtonsInit();

    //
    // Initialize the UART and show the application name on the UART.
    //
    ConfigureUART();
    UARTprintf("Watchdog example.\n\n");
    //
    // Show the state and offer some instructions to the user.
    //
    UARTprintf("Feeding Watchdog... Press the SW1 button to prevent reset.\n");

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Set GPIO PN0 as an output.  This drives an LED on the board that will
    // toggle when a watchdog interrupt is processed.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);

    timer_setup();
    watchdog_setup();

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Enable the timers.
    //
    TimerEnable(TIMER0_BASE, TIMER_A);

    //
    // Enable the watchdog timer.
    //
    WatchdogEnable(WATCHDOG0_BASE);

    //
    // Loop forever while the LED blinks as watchdog interrupts are handled.
    //
    uint8_t button_delta;

    while (1)
    {
        //
        // Poll for the SW1 button to be pressed.
        //
        uint8_t ui8Buttons = ButtonsPoll(&button_delta, 0);

        if ((ui8Buttons & USR_SW1) & button_delta)
        {
            SW1ButtonPressed();
        }
    }
}