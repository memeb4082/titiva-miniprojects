//*****************************************************************************
//
// light_opt3001.c - Example to use OPT3001
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
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
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "drivers/opt3001.h"

//*****************************************************************************
//
//Light Measurement with the OPT3001
//
// This example demonstrates the basic use of the OPT3001 light sensor
//
// The Sensors BoosterPack must be installed on BoosterPack 1 interface.
// Code changes will be needed if installed on BoosterPack 2 interface.
//
//
//*****************************************************************************


//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     1
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// Global variable for storage of actual system clock frequency.
//
//*****************************************************************************
volatile uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
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

    SysCtlDelay(g_ui32SysClock); // ~1 second delay at 120MHz to ensure UART initialises before using UARTprintf
}

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int main(void)
{
    bool      success;
    uint16_t  rawData = 0;
    float     convertedLux = 0;

    //
    // Configure the system frequency.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);


    //
    // Initialize the UART.
    //
    ConfigureUART();
    

    //
    // Clear the terminal and print the welcome message.
    //
    UARTprintf("OPT001 Example\n");

    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //
    // Enable interrupts to the processor.
    //
    IntMasterEnable();

    //Initialise sensor
    sensorOpt3001Init();

    // Test that sensor is set up correctly
    UARTprintf("Testing OPT3001 Sensor:\n");
    success = sensorOpt3001Test();

    //stay here until sensor is working
    while (!success) {
        SysCtlDelay(g_ui32SysClock);
        UARTprintf("Test Failed, Trying again\n");
        success = sensorOpt3001Test();
    }

    UARTprintf("All Tests Passed!\n\n");

 
    // Loop Forever
    while(1)
    {
        SysCtlDelay(g_ui32SysClock/100);

        //Read and convert OPT values
        success = sensorOpt3001Read(&rawData);

        if (success) {
            sensorOpt3001Convert(rawData, &convertedLux);

            // Construct Text
            int lux_int = (int)convertedLux;
            UARTprintf("Lux: %5d\n", lux_int);
        }

    }
}
