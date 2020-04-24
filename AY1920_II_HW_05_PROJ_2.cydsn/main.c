/**
* \brief Main source file for the I2C-Master project.
*
* In this project we set up a I2C master device with
* to understand the I2C protocol and communicate with a
* a I2C Slave device (LIS3DH Accelerometer).
*
*/

// Include required header files
#include "I2C_Interface.h"
#include "project.h"
#include "stdio.h"

/**
*   \brief 7-bit I2C address of the slave device.
*/
#define LIS3DH_DEVICE_ADDRESS 0x18

/**
*   \brief Address of the WHO AM I register
*/
#define LIS3DH_WHO_AM_I_REG_ADDR 0x0F

/**
*   \brief Address of the Status register
*/
#define LIS3DH_STATUS_REG 0x27

/**
*   \brief Address of the Control register 1
*/
#define LIS3DH_CTRL_REG1 0x20

/**
*   \brief Hex value to set normal mode at 100 Hz to the accelerator
*/
#define LIS3DH_NORMAL_MODE_100HZ_CTRL_REG1 0x57

/**
*   \brief Address of the Control register 4
*/
#define LIS3DH_CTRL_REG4 0x23  

// For the normal mode at 100 Hz and ±2.0 g FSR, Hex value is 0x00
#define LIS3DH_NORMAL_MODE_100HZ_CTRL_REG4 0x00

/**
*   \brief Address of the X-axis acceleration data output LSB register
*/
#define LIS3DH_OUT_X_L 0x28
/**
*   \brief Address of the Y-axis acceleration data output LSB register
*/
#define LIS3DH_OUT_Y_L 0x2A
/**
*   \brief Address of the Z-axis acceleration data output LSB register
*/
#define LIS3DH_OUT_Z_L 0x2C

//Thanks to the MultiRead function we don't need to specify the MSB registers Address

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    I2C_Peripheral_Start();
    UART_Debug_Start();
    
    CyDelay(5); //"The boot procedure is complete about 5 milliseconds after device power-up."
    
    // String to print out messages on the UART
    char message[50];

    // Check which devices are present on the I2C bus
    for (int i = 0 ; i < 128; i++)
    {
        if (I2C_Peripheral_IsDeviceConnected(i))
        {
            // print out the address is hex format
            sprintf(message, "Device 0x%02X is connected\r\n", i);
            UART_Debug_PutString(message); 
        }
        
    }
    
    /******************************************/
    /*            I2C Reading                 */
    /******************************************/
    
    /* Read WHO AM I REGISTER register */
    uint8_t who_am_i_reg;
    ErrorCode error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                                  LIS3DH_WHO_AM_I_REG_ADDR, 
                                                  &who_am_i_reg);
    if (error == NO_ERROR)
    {
        sprintf(message, "WHO AM I REG: 0x%02X [Expected: 0x33]\r\n", who_am_i_reg);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm\r\n");   
    }
    
    
    /******************************************/
    /*        Read Control Register 1         */
    /******************************************/
    uint8_t ctrl_reg1; 
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG1,
                                        &ctrl_reg1);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 1: 0x%02X\r\n", ctrl_reg1);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register 1\r\n");   
    }
     /******************************************/
    /*        Read Control Register 4        */
    /******************************************/
    uint8_t ctrl_reg4; 
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &ctrl_reg4);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 4: 0x%02X\r\n", ctrl_reg4);
        UART_Debug_PutString(message); 
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register 1\r\n");   
    }
    
    /******************************************/
    /*            I2C Writing                 */
    /******************************************/
    
    // Write value of register CTRL_REG1
    UART_Debug_PutString("\r\nWriting new values..\r\n");
    
    if (ctrl_reg1 != LIS3DH_NORMAL_MODE_100HZ_CTRL_REG1)
    {
        ctrl_reg1 = LIS3DH_NORMAL_MODE_100HZ_CTRL_REG1;
    
        error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                             LIS3DH_CTRL_REG1,
                                             ctrl_reg1);
    
        if (error == NO_ERROR)
        {
            sprintf(message, "CONTROL REGISTER 1 successfully written as: 0x%02X\r\n", ctrl_reg1);
            UART_Debug_PutString(message); 
        }
        else
        {
            UART_Debug_PutString("Error occurred during I2C comm to set control register 1\r\n");   
        }
    }
    // Write of register CTRL_REG4
    
    UART_Debug_PutString("\r\nWriting new values..\r\n");
    
    if (ctrl_reg4 != LIS3DH_NORMAL_MODE_100HZ_CTRL_REG4)
    {
        ctrl_reg4 = LIS3DH_NORMAL_MODE_100HZ_CTRL_REG4;
    
        error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                             LIS3DH_CTRL_REG4,
                                             ctrl_reg4);
    
        if (error == NO_ERROR)
        {
            sprintf(message, "CONTROL REGISTER 4 successfully written as: 0x%02X\r\n", ctrl_reg4);
            UART_Debug_PutString(message); 
        }
        else
        {
            UART_Debug_PutString("Error occurred during I2C comm to set control register 1\r\n");   
        }
    }

    
    
    uint8_t status_register;
    int16_t OutX,OutY,OutZ;
    uint8_t header = 0xA0;
    uint8_t footer = 0xC0;
    uint8_t OutArray[8]; 
    
    uint8_t AccData[6]; // Array of the acceleration data
   
    OutArray[0] = header;
    OutArray[7] = footer;
    
    for(;;)
    {
        //CyDelay(10); // we read data at 100 Hz (10ms of delay)
        
        //Read of the Status Register
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_STATUS_REG,
                                            &status_register);
        if(error == NO_ERROR)
        {
            if((status_register & 1<<3) == 8) //Control if ZYXDA is set to 1, 
                                              //in this case new set of data is available
            {
                if((status_register & 1<<7) == 128)// Control if ZYXOR is set to 1,
                                                   // in this case  a new set of 
                                                   //data has overwritten the previous set
               {
                    //The registers of the OUTPUT of X,Y,Z are consecutive so we use a Multi-Read 
                    error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                          LIS3DH_OUT_X_L,
                                                          6,
                                                          &AccData[0]);
                    
        
                    if(error == NO_ERROR)
                    {
                        OutX = (int16)((AccData[0] | (AccData[1]<<8)))>>6;
                        OutX = OutX*4; // Multiply the value for 4 because the sensitivity is 4 mg/digit
                        OutArray[1] = (uint8_t)(OutX & 0xFF);
                        OutArray[2] = (uint8_t)(OutX >> 8);
                        OutY = (int16)((AccData[2] | (AccData[3]<<8)))>>6;
                        OutY = OutY*4; // Multiply the value for 4 because the sensitivity is 4 mg/digit
                        OutArray[3] = (uint8_t)(OutY & 0xFF);
                        OutArray[4] = (uint8_t)(OutY >> 8);
                        OutZ = (int16)((AccData[4] | (AccData[5]<<8)))>>6;
                        OutZ = OutZ*4; // Multiply the value for 4 because the sensitivity is 4 mg/digit
                        OutArray[5] = (uint8_t)(OutZ & 0xFF);
                        OutArray[6] = (uint8_t)(OutZ >> 8);
          
                        UART_Debug_PutArray(OutArray, 8);
                    }
                }
            }
          }
    }
}

/* [] END OF FILE */
