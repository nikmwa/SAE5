/******************************************************************************
* File Name:   main.c
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "http_client.h"
#include "cy_http_client_api.h"
#include "cy_log.h"

/* Library for malloc and free */
#include "stdlib.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* btstack */
#include "wiced_bt_stack.h"

/* App utilities */
#include "app_bt_utils.h"
#include "app_bt.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#ifndef CYBSP_USER_LED2
#define CYBSP_USER_LED2 P10_0
#endif

#define UART_INPUT true

#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)

#define HTTPS_CLIENT_TASK_STACK_SIZE        (5 * 1024)
#define HTTPS_CLIENT_TASK_PRIORITY          (1)


/*******************************************************************
 * Function Prototypes
 ******************************************************************/

#if (UART_INPUT == true)
/* Tasks to handle UART */
static void rx_cback(void *handler_arg, cyhal_uart_event_t event); /* Callback for data received from UART */
static void uart_task(void *pvParameters);
#endif

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
/*UART task and Queue handles */
TaskHandle_t  UartTaskHandle = NULL;
QueueHandle_t xUARTQueue = 0;

/* HTTPS client task an Queue handles. */
TaskHandle_t https_client_task_handle;
QueueHandle_t httpQueue = 0;

// static cy_http_client_t https_client;

/*******************************************************************
 * Function Implementations
 ******************************************************************/

/*******************************************************************************
* Function Name: int main( void )
********************************************************************************/
int main(void)
{
    cy_rslt_t result ;

    /* Initialize the board support package */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                        CY_RETARGET_IO_BAUDRATE);

    /* Init QSPI and enable XIP to get the Wi-Fi firmware from the QSPI NOR flash */
    #if defined(CY_ENABLE_XIP_PROGRAM)
        const uint32_t bus_frequency = 50000000lu;

        cy_serial_flash_qspi_init(smifMemConfigs[0], CYBSP_QSPI_D0, CYBSP_QSPI_D1,
                                      CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
                                      CYBSP_QSPI_SCK, CYBSP_QSPI_SS, bus_frequency);

        cy_serial_flash_qspi_enable_xip(true);
    #endif

	/* Initialize pin to indicate scanning */
    cyhal_gpio_init(CYBSP_USER_LED2,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	// /* Log output level */
	// cy_log_set_all_levels(CY_LOG_DEBUG);

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize stack and register the callback function */
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

	/* Starts the HTTP client. */
    xTaskCreate(https_client_task, "HTTP Client", HTTPS_CLIENT_TASK_STACK_SIZE, (void *) &https_client,
               HTTPS_CLIENT_TASK_PRIORITY, &https_client_task_handle);

	/* Initialize HTTP client queue */
	httpQueue = xQueueCreate( 10, sizeof(uint8_t) );

	#if (UART_INPUT == true)
	/* Setup UART user input interface */
	xUARTQueue = xQueueCreate( 10, sizeof(uint8_t) );
	cyhal_uart_register_callback(&cy_retarget_io_uart_obj, rx_cback, NULL); /* Register UART Rx callback */
	cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY , 3, TRUE); /* Enable Rx interrupt */
	xTaskCreate (uart_task, "UartTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &UartTaskHandle); /* Start task */
	uint8_t helpCommand = '?';
	xQueueSend( xUARTQueue, &helpCommand, 0); /* Print out list of commands */
	#endif

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}



#if (UART_INPUT == true)
/*******************************************************************************
* Function Name: uart_task()
********************************************************************************
*
* Summary:
*   This function runs the UART task which processes the received commands via
*   Terminal.
*
* Parameters:
*   void *pvParameters                 Not used
*
* Return:
*   None
*
*******************************************************************************/
static void uart_task(void *pvParameters)
{
    uint8_t readbyte;
    for(;;)
    {
		/* Wait for a character to be sent from the UART ISR */
        if(pdPASS == xQueueReceive( xUARTQueue, &(readbyte), portMAX_DELAY))
        {
            switch (readbyte)
			{
				case 's':			// Turn on scanning
					wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY,TRUE,scanCallback);
					break;

				case 'S':			// Turn off scanning
					wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE,TRUE,scanCallback);
					break;

				case 'd': 			// Disconnect
					wiced_bt_gatt_disconnect(conn_id);
					break;

				case 'r':
					wiced_bt_gatt_client_send_read_handle(conn_id,ledChar.valHandle,0,&ledStatus,sizeof(ledStatus),GATT_AUTH_REQ_NONE);
					break;
				
				case 'n': 			//Set CCCD
					{
						uint8_t writeData[2] = {0};
						writeData[0]=GATT_CLIENT_CONFIG_NOTIFICATION;/* Values are sent little endian */
						writeAttribute(conn_id, counterChar.cccdHandle, 0, GATT_AUTH_REQ_SIGNED_NO_MITM, sizeof(uint16_t), writeData);
					}
					break;
				case 'N':			//Unset CCCD
					{
						uint8_t writeData[2] = {0};
						writeData[0]=GATT_CLIENT_CONFIG_NONE;/* Values are sent little endian */
						writeAttribute(conn_id, counterChar.cccdHandle, 0, GATT_AUTH_REQ_SIGNED_NO_MITM, sizeof(uint16_t), writeData);
					}
					break;
				case 'q': 			//Start service discovery
					startServiceDiscovery();
					break;
				case 'w':			//Start characteristic discovery
					startCharacteristicDiscovery();
					break;
				case 'e':
					startDescriptorDiscovery();
					break;
				case '0':			// LEDs off
					{
						uint8_t writeData[1];
						writeData[0] = readbyte-'0';
						writeAttribute(conn_id, ledChar.valHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint8_t), writeData);
					}
					break;
				case '1':			// LEDs blue
				case '2':			// LEDs red
				case '3':			// LEDs blue+red
				case '4':			// LEDs green
				case '5':			// LEDs blue+green
				case '6':			// LEDs red+green
				case '7':			// LEDs white
					{
					uint8_t writeData[1];
					writeData[0] = readbyte-'0';
					writeAttribute(conn_id, ledChar.valHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint8_t), writeData);
					}
					break;

				case 'm': 			// Send HTTP POST to server
					{
						cy_rslt_t result = send_http_example_request(https_client,CY_HTTP_CLIENT_METHOD_POST,HTTP_PATH);
						if( result != CY_RSLT_SUCCESS )
						{
							ERR_INFO(("Failed to send the http request.\n"));
						}
						else
						{
							printf("\r\n Successfully sent POST request to http server\r\n");
						}
					}
				
					break;
				default:
					printf( "Unrecognized command\r\n" );
					/* No break - fall through and display help */

				case '?':			// Help
					printf( "Commands:\r\n" );
					printf( "\t%c\tHelp (this message)\r\n", '?' );
					printf( "\t%c\tStart scanning\r\n", 's' );
					printf( "\t%c\tStop scanning\r\n", 'S' );
					printf( "\t%c\tDisconnect\r\n", 'd' );
					printf( "\t%c\tLED on\r\n", '7' );
					printf( "\t%c\tLED off\r\n", '0' );
					printf( "\t%c\tRead LED status\r\n", 'r' );
					printf( "\t%c\tStart notifying button count\r\n", 'n' );
					printf( "\t%c\tStop notifying button count\r\n", 'N' );
					printf( "\t%c\tStart service discovery\r\n", 'q' );
					printf( "\t%c\tStart characteristic discovery\r\n", 'w' );
					printf( "\t%c\tStart descriptor discovery\r\n", 'e' );
					printf( "\t%c\tSend test http request POST\r\n", 'm' );
					printf( "\r\n" );
					break;
			}
        }
    }
}


/*******************************************************************************
* Function Name: rx_cback()
********************************************************************************
*
* Summary:
*   This function gets a character from the UART and sends it to the UART
*   task for processing
*
* Parameters:
*   void *handler_arg:                 Not used
*   cyhal_uart_event_t event:          Not used
*
* Return:
*   None
*
*******************************************************************************/
void rx_cback(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    uint8_t readbyte;
	cy_rslt_t status;
    BaseType_t xYieldRequired = pdFALSE;

    /* Read one byte from the buffer with a 1ms timeout */
    status = cyhal_uart_getc(&cy_retarget_io_uart_obj , &readbyte, 1);

    /* If a character was received, send it to the UART task */
	if(CY_RSLT_SUCCESS == status)
	{
    	xQueueSendFromISR( xUARTQueue, &readbyte, &xYieldRequired);
	}

	/* Yield current task if a higher priority task is now unblocked */
	portYIELD_FROM_ISR(xYieldRequired);
}
#endif

/* [] END OF FILE */
