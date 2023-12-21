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

/* Library for malloc and free */
#include "stdlib.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* btstack */
#include "wiced_bt_stack.h"

/* Bluetooth app utilities and functions */
#include "app_bt_utils.h"
#include "app_bt.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"

/* HTTP client functions */
#include "cy_http_client_api.h"

/* Fonctions MQTT*/
#include "mqtt_client_config.h"
#include "mqtt_task.h"

#include "publisher_task.h"
#include "subscriber_task.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)

#define HTTPS_CLIENT_TASK_STACK_SIZE        (5 * 1024)
#define HTTPS_CLIENT_TASK_PRIORITY          (1)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/

/* Tasks to handle UART */
static void rx_cback(void *handler_arg, cyhal_uart_event_t event); /* Callback for data received from UART */
static void uart_task(void *pvParameters);

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
/*UART task and Queue handles */
static TaskHandle_t  UartTaskHandle = NULL;
static QueueHandle_t xUARTQueue = 0;

/* HTTP client task handle */
static TaskHandle_t HTTPClientTaskHandle;

/*******************************************************************
 * Function Implementations
 ******************************************************************/

/*******************************************************************************
* Function Name: int main( void )
********************************************************************************/

int main()
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */
    /* Initialize the board support package. */
    
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* To avoid compiler warnings. */
    (void) result;

    /* Enable global interrupts. */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

#if defined(CY_DEVICE_PSOC6A512K)
    /* Initialize the QSPI serial NOR flash with clock frequency of 50 MHz. */
    const uint32_t bus_frequency = 50000000lu;
    cy_serial_flash_qspi_init(smifMemConfigs[0], CYBSP_QSPI_D0, CYBSP_QSPI_D1,
                                  CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
                                  CYBSP_QSPI_SCK, CYBSP_QSPI_SS, bus_frequency);

    /* Enable the XIP mode to get the Wi-Fi firmware from the external flash. */
    cy_serial_flash_qspi_enable_xip(true);
#endif

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H");
    printf("===============================================================\n");
#if defined(COMPONENT_CM0P)
    printf("CE229889 - MQTT Client running on CM0+\n");
#endif

#if defined(COMPONENT_CM4)
    printf("CE229889 - MQTT Client running on CM4\n");
#endif

#if defined(COMPONENT_CM7)
    printf("CE229889 - MQTT Client running on CM7\n");
#endif
    printf("===============================================================\n\n");


    /* Create the MQTT Client task. */
    xTaskCreate(mqtt_client_task, "MQTT Client task", MQTT_CLIENT_TASK_STACK_SIZE,
                NULL, MQTT_CLIENT_TASK_PRIORITY, NULL);

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never get here. */
    CY_ASSERT(0);
}




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
					wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY,TRUE,BLEScanCallback);
					break;

				case 'S':			// Turn off scanning
					wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE,TRUE,BLEScanCallback);
					break;

				case 'd': 			// Disconnect
					wiced_bt_gatt_disconnect(bt_conn_id);
					break;

				case 'r': 			// Read LED status
					wiced_bt_gatt_client_send_read_handle(bt_conn_id,ledChar.valHandle,0,&ledStatus,32*sizeof(uint8_t),GATT_AUTH_REQ_NONE);
					break;
				
				case 'n': 			//Set CCCD
					{
						uint8_t writeData[2] = {0};
						writeData[0]=GATT_CLIENT_CONFIG_NOTIFICATION;/* Values are sent little endian */
						writeAttribute(bt_conn_id, counterChar.cccdHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint16_t), writeData);
					}
					break;
				case 'N':			//Unset CCCD
					{
						uint8_t writeData[2] = {0};
						writeData[0]=GATT_CLIENT_CONFIG_NONE;/* Values are sent little endian */
						writeAttribute(bt_conn_id, counterChar.cccdHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint16_t), writeData);
					}
					break;
				case 'q': 			//Start service discovery
					startBTServiceDiscovery();
					break;
				case 'w':			//Start characteristic discovery
					startBTCharacteristicDiscovery();
					break;
				case 'e':			////Start descriptor discovery
					startBTDescriptorDiscovery();
					break;
				case '0':			// LED off
					{
						uint8_t writeData[1];
						writeData[0] = readbyte-'0';
						writeAttribute(bt_conn_id, ledChar.valHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint8_t), writeData);
					}
					break;
				case '7':			// LED on
					{
					uint8_t writeData[1];
					writeData[0] = readbyte-'0';
					writeAttribute(bt_conn_id, ledChar.valHandle, 0, GATT_AUTH_REQ_NONE, sizeof(uint8_t), writeData);
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

/* [] END OF FILE */
