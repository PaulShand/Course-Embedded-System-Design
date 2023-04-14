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

#define GPIO_INTERRUPT_PRIORITY (7u)

bool o = true;
bool b = true;
bool cu = true;
bool m = true;

uint8_t blue = 0;
uint8_t green = 0;
uint8_t red = 0;
uint8_t pick = 0;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/
//https://iotexpert.com/i2c-detect-with-psoc-6/


/*******************************************************************************
* Function Name: main
*
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

void handler(void);

cy_stc_sysint_t intrCfg =
    {

            .intrSrc = ioss_interrupts_gpio_0_IRQn            /* CM0+ interrupt is NVIC #7 */

    };


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

    Cy_SysInt_Init(&intrCfg, &handler);
        NVIC_EnableIRQ(intrCfg.intrSrc);


        Cy_SysAnalog_Enable();

            cy_en_sar_status_t status1;
            status1 = Cy_SAR_Init(SAR, &ADC_config);
            if (CY_SAR_SUCCESS == status1)
            {
            /* Turn on the SAR hardware. */
            	Cy_SAR_Enable(SAR);
            /* Begin continuous conversions. */
                Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);
            }


    /* Enable global interrupts */
    __enable_irq();
    printf("\x1b[2J\x1b[;H");
    printf("CSE121 Lab 3.3 DFR0554 Library\r\n");
    LCD_Start(I2C_HW, &I2C_context);


    LCD_Clear();


    LCD_Print("RED  GREEN  BLUE");


	int cok;
	char colorn[16];
	for (;;)
	{
		LCD_SetRGB(red, green, blue);
		sprintf(colorn, "%03d  %03d    %03d",red, green, blue);
		LCD_SetCursor(0, 1);
		CyDelay(5);
		LCD_Print(colorn);
		CyDelay(5);
		uint16_t res = Cy_SAR_GetResult16(SAR, 0U);
		//printf("%d\r\n", res);
		if((res < 2050) && (res > 1)){
			cok = res / 8;
		}
		else if(res > 65000){
			cok = 0;
		}
		if(pick == 0){
			red = cok;
		}
		else if(pick == 1){
			green = cok;
		}
		else if(pick == 2){
			blue = cok;
		}
		printf("RED  GREEN  BLUE:\r\n");
		printf("%s\r\n", colorn);



    }

}

void handler(void)
{
    if(pick == 0){
    	pick = 1;
    	printf("Switching to GREEN...\r\n");
    }
    else if(pick == 1){
    	pick = 2;
    	printf("Switching to BLUE...\r\n");
    }
    else if(pick == 2){
    	pick = 0;
    	printf("Switching to RED...\r\n");
    }
    Cy_GPIO_ClearInterrupt(BUTTON_PORT,BUTTON_NUM);
}
/* [] END OF FILE */
