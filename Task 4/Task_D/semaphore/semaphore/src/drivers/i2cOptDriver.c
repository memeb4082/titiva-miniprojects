/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        23 March 2019
*  Revision:       2.0
*
*  Description:    i2c Driver for use with opt3001.c and the TI OP3001 Optical Sensor
*************************************************************************************************/

// ----------------------- Includes -----------------------
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "i2cOptDriver.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"


void I2C0IntHandler(void);


extern SemaphoreHandle_t i2cSemaphore;

/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE)) { }
    // UARTprintf("Here1\n");
    // xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    // UARTprintf("Here2\n");

    // Send Data
    I2CMasterDataPut(I2C0_BASE, data[0]);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_BASE)) { }
    // xSemaphoreTake(i2cSemaphore, portMAX_DELAY);

    I2CMasterDataPut(I2C0_BASE, data[1]);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE)) { }
    // xSemaphoreTake(i2cSemaphore, portMAX_DELAY);

    return true;
}



/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over i2c to specify register being read from
 * Reads three bytes from i2c slave. The third is redundant but
 * helps to flush the i2c register
 * Stores first two received bytes into *data
 */
bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address and change I2C to write
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C0_BASE, ui8Reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE)) { }
    // xSemaphoreTake(i2cSemaphore, portMAX_DELAY);

    // Load device slave address and change I2C to read
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8Addr, true);

    // Read two bytes from I2C
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE)) { }
    // xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    data[0] = I2CMasterDataGet(I2C0_BASE);

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE)) { }
    // xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
    data[1] = I2CMasterDataGet(I2C0_BASE);

    return true;
}


void I2C0IntHandler(void){
    // uint32_t status;
    // Read and clear the interrupt status
    // status = I2CMasterIntStatus(I2C0_BASE, true);
    I2CMasterIntClear(I2C0_BASE);
    // UARTprintf("Here3\n");
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(i2cSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

