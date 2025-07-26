//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
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
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/tm4c1294ncpdt.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
    while (1)
        ;
}
#endif

//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int main(void)
{
    volatile uint32_t ui32Loop;

    int LED_D3 = 0b00010000; // or 0x0
    int LED_D4 = 0b00000001; // or 0x01
    int LED_D2 = 0b00000001; // or 0x01
    int LED_D1 = 0b00000010; // or 0x02

    //
    // Enable the GPIO port that is used for the on-board LED. SYSCTL_RCGCGPIO_R12
    // SYSCTL_RCGCGPIO_R8 is the other port enabled (bitwise so they both work)
    //
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R12 | SYSCTL_RCGCGPIO_R5;

    // Short delay to allow peripheral to enable
    for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
    {
    }

    // Enable GPIO pin for LED D2 (PN0)
    GPIO_PORTN_DEN_R = (LED_D2 | LED_D1);

    // Enable GPIO pin for LED 3/4
    GPIO_PORTF_AHB_DEN_R = (LED_D3 | LED_D4);

    // Set the direction as output, and enable the GPIO pin for digital function
    GPIO_PORTN_DIR_R = (LED_D2 | LED_D1);

    // Set the direction as output, and enable the GPIO pin for digital function for 3/44
    GPIO_PORTF_AHB_DIR_R = (LED_D3 | LED_D4);

    //
    // Loop forever.
    //
    while (1)
    {

        GPIO_PORTN_DATA_R |= (LED_D1);
        for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        {
        }

        GPIO_PORTN_DATA_R &= ~(LED_D1);
        GPIO_PORTN_DATA_R |= (LED_D2);
        for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        {
        }

        GPIO_PORTN_DATA_R &= ~(LED_D2);
        GPIO_PORTF_AHB_DATA_R |= (LED_D3);
        for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        {
        }

        GPIO_PORTF_AHB_DATA_R &= ~(LED_D3);
        GPIO_PORTF_AHB_DATA_R |= (LED_D4);
        for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        {
        }
        GPIO_PORTF_AHB_DATA_R &= ~(LED_D4);

        //---------------
        //
        // Turn on the LED.
        //
        // GPIO_PORTN_DATA_R |= (LED_D2|LED_D1);
        // GPIO_PORTF_AHB_DATA_R |= (LED_D3|LED_D4);

        // //
        // // Delay for a bit.
        // //
        // for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        // {
        // }

        // //
        // // Turn off the LED.
        // //
        // GPIO_PORTN_DATA_R &= ~(LED_D2|LED_D1);
        // GPIO_PORTF_AHB_DATA_R &= ~(LED_D3|LED_D4);

        // //
        // // Delay for a bit.
        // //
        // for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        // {
        // }
    }
}
