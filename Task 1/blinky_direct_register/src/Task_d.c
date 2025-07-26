
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/tm4c1294ncpdt.h"

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
// BUTTONS
//
//*****************************************************************************
int main(void)
{
    volatile uint32_t ui32Loop;

    int LED_D3 = 0b00010000; // or 0x10
    int LED_D4 = 0b00000001; // or 0x01
    int LED_D2 = 0b00000001; // or 0x01
    int LED_D1 = 0b00000010; // or 0x02

    int PJ0 = 0b00000001; // pin 0
    int PJ1 = 0b00000010; // pin 1

    //
    // Enable the GPIO port that is used for the on-board LED. SYSCTL_RCGCGPIO_R12
    // SYSCTL_RCGCGPIO_R8 is the other port enabled (bitwise so they both work)
    //
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    // Short delay to allow peripheral to enable
    for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
    {
    }

    // Buttons!
    // Enables the pins function
    GPIO_PORTJ_AHB_DEN_R |= (PJ0 | PJ1);

    // Sets the pins as input (0 in input, 1 = output)
    GPIO_PORTJ_AHB_DIR_R &= ~(PJ0 | PJ1);

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
        // they ar epull up resistors, so will read 1 when non_pressed
        if ((GPIO_PORTJ_AHB_DATA_R & PJ0) == 0)
        {
            GPIO_PORTN_DATA_R |= (LED_D1);
        }
        else
        {
            GPIO_PORTN_DATA_R &= ~(LED_D1);
        }

        if ((GPIO_PORTJ_AHB_DATA_R & PJ1) == 0)
        {
            GPIO_PORTN_DATA_R |= (LED_D2);
        }
        else
        {
            GPIO_PORTN_DATA_R &= ~(LED_D2);
        }

        // GPIO_PORTN_DATA_R |= (LED_D1);
        // for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        // {
        // }

        // GPIO_PORTN_DATA_R &= ~(LED_D1);
        // GPIO_PORTN_DATA_R |= (LED_D2);
        // for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        // {
        // }

        // GPIO_PORTN_DATA_R &= ~(LED_D2);
        // GPIO_PORTF_AHB_DATA_R |= (LED_D3);
        // for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        // {
        // }

        // GPIO_PORTF_AHB_DATA_R &= ~(LED_D3);
        // GPIO_PORTF_AHB_DATA_R |= (LED_D4);
        // for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
        // {
        // }
        // GPIO_PORTF_AHB_DATA_R &= ~(LED_D4);

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
