/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "InterruptRoutines.h"

uint8 Flag_Read = 0;  // Initialitazion of the flag

CY_ISR(Custom_ISR)
{  
    Timer_ReadStatusRegister();
     Flag_Read = 1;  // flag high at every 10ms
    
}
/* [] END OF FILE */
