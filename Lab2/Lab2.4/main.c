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
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include <stdio.h>
/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_INTERRUPT_PRIORITY (7u)

/*******************************************************************************
* Global Variables
*******************************************************************************/


int compare = 500;
int dir = 1;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/

void handler(void);

cy_stc_sysint_t intrCfg =
    {

            .intrSrc = ioss_interrupts_gpio_0_IRQn            /* CM0+ interrupt is NVIC #7 */

    };
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



//    cyhal_gpio_register_callback(BUTTON, gpio_interrupt_handler, NULL);
    Cy_SysInt_Init(&intrCfg, &handler);
    NVIC_EnableIRQ(intrCfg.intrSrc);
    //Cy_GPIO_SetInterruptMask(BUTTON_PORT, BUTTON_NUM, 7UL);




    cy_en_sysanalog_status_t status;
    status = Cy_SysAnalog_Init(&pass_0_aref_0_config);


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




//    cyhal_gpio_enable_event(BUTTON, CYHAL_GPIO_IRQ_FALL, GPIO_INTERRUPT_PRIORITY, true);
//        /* Enable global interrupts */
    __enable_irq();
    printf("\x1b[2J\x1b[;H");

    printf("Lab 2.3\r\n");



    Cy_TCPWM_PWM_Init(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM, &tcpwm_0_cnt_5_config);
    Cy_TCPWM_PWM_Enable(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_NUM);
    Cy_TCPWM_TriggerStart(tcpwm_0_cnt_5_HW, tcpwm_0_cnt_5_MASK);



    Cy_TCPWM_PWM_SetCompare0(TCPWM0, tcpwm_0_cnt_5_NUM, 500);
    setvbuf(stdin, NULL, _IONBF, 0);


    int cok;
    for (;;)
    {
    	uint16_t res = Cy_SAR_GetResult16(SAR, 0U);
    	printf("%d\r\n", res);
    	if((res < 2000) && (res > 4)){
    		cok = res / 2;
    	}
    	Cy_TCPWM_PWM_SetCompare0(TCPWM0, tcpwm_0_cnt_5_NUM, cok);
    	//printf("%d\r\n", compare);
    }
}


void handler(void)
{
    //printf("made it\r\n");

    if (compare == 1000){
    	dir = 1;
    }
    else if (compare == 0){
    	dir = 0;
    }

    if (dir == 1){
    	compare -= 50;
    }
    else if (dir == 0){
    	compare += 50;
    }
    //printf("%d\r\n", compare);
    Cy_TCPWM_PWM_SetCompare0(TCPWM0, tcpwm_0_cnt_5_NUM, compare);
    Cy_GPIO_ClearInterrupt(BUTTON_PORT,BUTTON_NUM);
}

/* [] END OF FILE */
