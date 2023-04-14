/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include "DFR0554.h"

#include <string.h>

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_uart_context_t STDIO_context;
cy_stc_scb_i2c_context_t I2C_context;

bool o = true;
bool b = true;
bool cu = true;
bool m = true;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }


    init_cycfg_all();

    setvbuf( stdin, NULL, _IONBF, 0 ); // Turn off stdin buffering
    Cy_SCB_I2C_Init(I2C_HW,&I2C_config,&I2C_context);
    Cy_SCB_I2C_Enable(I2C_HW);

    Cy_TCPWM_Counter_Init(Timer_HW, Timer_NUM, &Timer_config);
    Cy_TCPWM_Counter_Enable(Timer_HW, Timer_NUM);



    /* Enable global interrupts */
    __enable_irq();
    printf("\x1b[2J\x1b[;H");
    printf("CSE121 Lab 4.1 DFR0554 Library\r\n");
    LCD_Start(I2C_HW, &I2C_context);



    LCD_SetColor(White);

    uint32_t count;

    char str[10];

    for (;;) {

    	//CyDelay(50);
    	printf("try\r\n");
    	Cy_GPIO_Set(Trigger_PORT,Trigger_NUM);
    	CyDelay(10);
    	Cy_GPIO_Clr(Trigger_PORT,Trigger_NUM);

    	if(0 == Cy_GPIO_Read(Echo_PORT, Echo_NUM)){
    		while(1){
				if(1 == Cy_GPIO_Read(Echo_PORT, Echo_NUM)){
					Cy_TCPWM_Counter_SetCounter(Timer_HW,Timer_NUM,0);
					Cy_TCPWM_TriggerStart(Timer_HW,Timer_MASK);
					printf("on\r\n");
					break;
				}
			}
    	}

    	if(1 == Cy_GPIO_Read(Echo_PORT, Echo_NUM)){
    		while(1){
				if(0 == Cy_GPIO_Read(Echo_PORT, Echo_NUM)){
					count = Cy_TCPWM_Counter_GetCounter(Timer_HW, Timer_NUM);
					printf("off: %d\r\n",count);
					break;
				}
			}
    	}

    	Cy_TCPWM_Counter_SetCounter(Timer_HW,Timer_NUM,0);
    	count = count / 32;
    	LCD_SetCursor(0, 0);
    	sprintf(str,"  %03lu cm",count);

    	LCD_Print(str);

    }

}
/* [] END OF FILE */
