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


    /* Enable global interrupts */
    __enable_irq();
    printf("\x1b[2J\x1b[;H");
    printf("CSE121 Lab 3.2 DFR0554 Library\r\n");
    LCD_Start(I2C_HW, &I2C_context);



    LCD_SetColor(White);
    LCD_Print("Hello CSE121");
    printf("> Usage: 'o' Switch Display On/Off\r\n");
        	printf("  'b' Switch Blinking On/Off\r\n");
        	printf("  'c' Switch Cursor On/Off\r\n");
        	printf("  'm' Move Cursor\r\n");
        	printf("  'l' Scroll Left\r\n");
        	printf("  'r' Scroll Right\r\n");
        	printf("  'W' Set Color White\r\n");
        	printf("  'R' Set Color Red\r\n");
        	printf("  'G' Set Color Green\r\n");
        	printf("  'B' Set Color Blue\r\n");
        	printf("  'P' Set Color Pink\r\n");
        	printf("  'h' Show Usage\r\n");
    for (;;) {

    	char c = getchar();
    	printf("> You Entered: %c\r\n", c);
    	switch(c)
    	{
    	    case 'o':

    	    	if(o == true){
    	    		printf("Turning Off Display...\r\n");
    	    		LCD_Display(Off);
    	    		o = false;
    	    	}
    	    	else{
    	    		printf("Turning On Display...\r\n");
    	    		LCD_Display(On);
    	    		o = true;
    	    	}
    	    break;

    	    case 'b':

				if(b == true){
					printf("Start Blinking...\r\n");
					LCD_Blink(On);
					b = false;
				}
				else{
					printf("Stop Blinking...\r\n");
					LCD_Blink(Off);
					b = true;
				}
    	    break;

    	    case 'c':

				if(cu == true){

					printf("Show Cursor...\r\n");
					LCD_Cursor(On);
					cu = false;
				}
				else{
					printf("Hide Cursor...\r\n");
					LCD_Cursor(Off);
					cu = true;
				}
    	    break;

    	    case 'm':

				if(m == true){
					printf("Moving Cursor Down...\r\n");
					LCD_SetCursor(0,1);
					m = false;
				}
				else{
					printf("Moving Cursor Up...\r\n");
					LCD_SetCursor(12,0);
					m = true;
				}
    	     break;

    	    case 'l':
    	    	printf("Scroll Left...\r\n");
    	    	LCD_Scroll(Left);

			 break;

    	    case 'r':
				printf("Scroll Right...\r\n");
				LCD_Scroll(Right);

    	    break;

    	    case 'W':
				printf("Set LED Color to White...\r\n");
				LCD_SetColor(White);

			break;

    	    case 'R':
				printf("Set LED Color to Red...\r\n");
				LCD_SetColor(Red);

			break;
    	    case 'G':
				printf("Set LED Color to Green...\r\n");
				LCD_SetColor(Green);

			break;
    	    case 'B':
				printf("Set LED Color to Blue...\r\n");
				LCD_SetColor(Blue);

			break;
    	    case 'P':
				printf("Set LED Color to Pink...\r\n");
				LCD_SetRGB(255, 125, 232);

			break;
    	    case 'h':
    	    	printf("> Usage: 'o' Switch Display On/Off\r\n");
				printf("  'b' Switch Blinking On/Off\r\n");
				printf("  'c' Switch Cursor On/Off\r\n");
				printf("  'm' Move Cursor\r\n");
				printf("  'l' Scroll Left\r\n");
				printf("  'r' Scroll Right\r\n");
				printf("  'W' Set Color White\r\n");
				printf("  'R' Set Color Red\r\n");
				printf("  'G' Set Color Green\r\n");
				printf("  'B' Set Color Blue\r\n");
				printf("  'P' Set Color Pink\r\n");
				printf("  'h' Show Usage\r\n");

			break;

    	}
    }

}
/* [] END OF FILE */
