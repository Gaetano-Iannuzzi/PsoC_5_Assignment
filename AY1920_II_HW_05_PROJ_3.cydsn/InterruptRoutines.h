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
#ifndef __INTERRUPT_ROUTINES_H
    // Header guard
    
   #define __INTERRUPT_ROUTINES_H
   #include "project.h"
    
   /*
    // Definition of a global flag for the constant rate read
   */
   extern uint8 Flag_Read; 
    
   CY_ISR_PROTO(Custom_ISR);

#endif

/* [] END OF FILE */
