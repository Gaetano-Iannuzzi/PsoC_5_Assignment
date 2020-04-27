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
#include "InterruptRoutines.h"
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
*   \brief Hex value to set high resolution mode at 100 Hz to the accelerator
*/
#define LIS3DH_HIGH_RESOLUTION_MODE_100HZ_CTRL_REG1 0x57

/**
*   \brief Address of the Control register 4
*/
#define LIS3DH_CTRL_REG4 0x23  

/**
*   \brief Hex value to set High Resolution mode at 100 Hz in the ±4.0 g FSR to the accelerator (BDU =1)
*/
#define LIS3DH_HIGH_RESOLUTION_MODE_100HZ_CTRL_REG4 0x98

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
    
      //Write CTRL_REG1  
    UART_Debug_PutString("\r\nWriting new values..\r\n");
    
    if (ctrl_reg1 != LIS3DH_HIGH_RESOLUTION_MODE_100HZ_CTRL_REG1)
    {
        ctrl_reg1 = LIS3DH_HIGH_RESOLUTION_MODE_100HZ_CTRL_REG1;
    
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
    //Write CTRL_REG4
    
     UART_Debug_PutString("\r\nWriting new values..\r\n");
    
    if (ctrl_reg4 != LIS3DH_HIGH_RESOLUTION_MODE_100HZ_CTRL_REG4)
    {
        ctrl_reg4 = LIS3DH_HIGH_RESOLUTION_MODE_100HZ_CTRL_REG4;
    
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
    int16_t OutX,OutY,OutZ;  //int16 values of acceleration
    int32_t OutX32,OutY32,OutZ32; //int32 values of acceleration after the cast of the floating point
    float32 AccX,AccY,AccZ; //floating point values in m/s^2
    uint8_t AccData[6]; // Array of the acceleration data
    
    
    uint8_t OutArray[14]; // In this case we have 4 byte for every axis + 1 header +  1 tail
    uint8_t header = 0xA0;
    uint8_t footer = 0xC0;
    OutArray[0] = header;
    OutArray[13] = footer;
    
    Timer_Start();  //Timer Start
    isr_Read_StartEx(Custom_ISR); //Start of the ISR
    
    for(;;)
    {
        if(Flag_Read != 0)  //ISR for read data at every 10ms
        { 
            //Read of the Status Register 
            error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                                LIS3DH_STATUS_REG,
                                                &status_register);
            if(error == NO_ERROR)
            {
                if((status_register & 1<<3) == 8) //Control if ZYXDA is set to 1, 
                                                  //in this case new set of data is available
               {
                    //The registers of the OUTPUT of X,Y,Z are consecutive so we use a Multi-Read 
                    error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                             LIS3DH_OUT_X_L,
                                                             6,
                                                             &AccData[0]);
                    
                    if(error == NO_ERROR)
                    {
                        OutX = (int16)((AccData[0] | (AccData[1]<<8)))>>4; //We have 12-bit of data
                        AccX=  OutX*2*9.806*0.001; // Multiply the value for 2 because the sensitivity is 2 mg/digit and 9.806*0.001 m/s^2
                        OutX32 =  AccX*1000; //Cast the floating point value to an int32 
                                                   //without loosing information of 3 decimals using the multiplication by 1000
                        OutArray[1] = (uint8_t)(OutX32 & 0xFF);
                        OutArray[2] = (uint8_t)(OutX32 >>8);
                        OutArray[3] = (uint8_t)(OutX32 >>16);
                        OutArray[4] = (uint8_t)(OutX32 >>24);
                        
                        OutY = (int16)((AccData[2] | (AccData[3]<<8)))>>4;
                        AccY = OutY*2*9.806*0.001; // Multiply the value for  2 because the sensitivity is 2 mg/digit and 9.806*0.001 m/s^2
                        OutY32 = AccY*1000; //Cast the floating point value to an int32 
                                                  //without loosing information of 3 decimals using the multiplication by 1000  
                        OutArray[5] = (uint8_t)(OutY32 & 0xFF);
                        OutArray[6] = (uint8_t)(OutY32 >> 8);
                        OutArray[7] = (uint8_t)(OutY32 >>16);
                        OutArray[8] = (uint8_t)(OutY32 >>24);
                        
                        OutZ = (int16)((AccData[4] | (AccData[5]<<8)))>>4;
                        AccZ = OutZ*2*9.806*0.001; // Multiply the value for 2 because the sensitivity is 2 mg/digit and 9.806*0.001 m/s^2
                        OutZ32 = AccZ*1000;//Cast the floating point value to an int32 
                                               //without loosing information of 3 decimals using the multiplication by 1000  
                        OutArray[9] = (uint8_t)(OutZ32 & 0xFF);
                        OutArray[10] =(uint8_t)(OutZ32 >> 8);
                        OutArray[11] = (uint8_t)(OutZ32 >>16);
                        OutArray[12] = (uint8_t)(OutZ32 >>24);
          
                        UART_Debug_PutArray(OutArray, 14);  //Send array to the Uart (values in [m/s^2])
                        
                        Flag_Read = 0;  //Set the ISR flag to 0
                    }
                }
            }
         }
    }
    
    
}

/* [] END OF FILE */