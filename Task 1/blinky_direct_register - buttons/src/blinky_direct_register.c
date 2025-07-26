
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

int LED_D3 = 0b00010000; // or 0x10
int LED_D4 = 0b00000001; // or 0x01
int LED_D2 = 0b00000001; // or 0x01
int LED_D1 = 0b00000010; // or 0x02

int PJ0 = 0b00000001; // pin 0
int PJ1 = 0b00000010; // pin 1

void all_off(void)
{
    GPIO_PORTN_DATA_R &= ~(LED_D1 | LED_D2);
    GPIO_PORTF_AHB_DATA_R &= ~(LED_D3 | LED_D4);
}

int main(void)
{
    volatile uint32_t ui32Loop;

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

    // Enable pull-up resistors for PJ0 and PJ1
    GPIO_PORTJ_AHB_PUR_R |= (PJ0 | PJ1); // Enable pull-up resistors on PJ0 and PJ1

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

    // 0 equals the normal way, 1 equals the flipped
    bool direction_norm = true;

    int light = 1;

    while (1)
    {
        // they ar epull up resistors, so will read 1 when non_pressed
        //Not written for both buttons to be pressed, would be easy to change, but thats not what I'm doing here

        if ((GPIO_PORTJ_AHB_DATA_R & PJ1) == 0)
        {
            direction_norm = true;
        }

        if ((GPIO_PORTJ_AHB_DATA_R & PJ0) == 0)
        {
            direction_norm = false;
        }

        switch (light)
        {
        // starting at 1 for d1-d4
        case 1:
            all_off();
            GPIO_PORTN_DATA_R |= (LED_D1);
            if (direction_norm == true)
            {
                light = 2;
            }
            else
            {
                light = 4;
            }
            for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
            {
            }
            break;
        case 2:
            all_off();
            GPIO_PORTN_DATA_R |= (LED_D2);
            if (direction_norm == true)
            {
                light = 3;
            }
            else
            {
                light = 1;
            }
            for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
            {
            }
            break;
        case 3:
            all_off();
            GPIO_PORTF_AHB_DATA_R |= (LED_D3);
            if (direction_norm == true)
            {
                light = 4;
            }
            else
            {
                light = 2;
            }
            for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
            {
            }
            break;
        case 4:
            all_off();
            GPIO_PORTF_AHB_DATA_R |= (LED_D4);
            if (direction_norm == true)
            {
                light = 1;
            }
            else
            {
                light = 3;
            }
            for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
            {
            }
            break;
        }

        // if ((GPIO_PORTJ_AHB_DATA_R & PJ0) == 0)
        // {
        //     GPIO_PORTN_DATA_R |= (LED_D1);
        // }
        // else
        // {
        //     GPIO_PORTN_DATA_R &= ~(LED_D1);
        // }

        // if ((GPIO_PORTJ_AHB_DATA_R & PJ1) == 0)
        // {
        //     GPIO_PORTN_DATA_R |= (LED_D2);
        // }
        // else
        // {
        //     GPIO_PORTN_DATA_R &= ~(LED_D2);
        // }

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
