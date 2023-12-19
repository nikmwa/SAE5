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
#include "http_client.h"
#include "cy_http_client_api.h"

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

	/* Initialize pin to indicate scanning */
    cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Initialize BT stack and register the callback function */
    wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

	/* Starts the HTTP client. */
    xTaskCreate(http_client_task, "HTTP Client", HTTPS_CLIENT_TASK_STACK_SIZE, (void *) &http_client,
                HTTPS_CLIENT_TASK_PRIORITY, &HTTPClientTaskHandle);

	/* Setup UART user input interface */
	xUARTQueue = xQueueCreate( 10, sizeof(uint8_t) );
	cyhal_uart_register_callback(&cy_retarget_io_uart_obj, rx_cback, NULL); /* Register UART Rx callback */
	cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY , 3, TRUE); /* Enable Rx interrupt */
	xTaskCreate (uart_task, "UartTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &UartTaskHandle); /* Start task */
	uint8_t helpCommand = '?';
	xQueueSend( xUARTQueue, &helpCommand, 0); /* Print out list of commands */

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}

//TODO list:

//Handle multiple devices
//1. Create a list of devices (array of devices) that you can add to
//2. When you find a device, add it to the list (BLEScanCallback)
//3. When you want to connect to a device, use the list to find the device you want to connect to (modify BLEScanCallback to only update the list, not connect) (write a function to connect to a device)
//4. When you want to disconnect from a device, use the list to find the device you want to disconnect from (modify app_bt_connect_event_handler to cycle through the list and disconnect from the device you want to disconnect from) (may b need to write a function to disconnect from a device)


//when connected to a device to read/write a characteristic you need to:
//1. once connected, start service discovery (startBTServiceDiscovery)
//2. once service discovery is done, start characteristic discovery (startBTCharacteristicDiscovery)
//3. once characteristic discovery is done, start descriptor discovery (startBTDescriptorDiscovery)
//4. find in all the UUIDs the one that matches the UUID of the characteristic you want to read/write (sensors)
//5. do something with the value





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
				case 'm':
					{
						send_http_example_request(http_client, CY_HTTP_CLIENT_METHOD_POST,HTTP_PATH);
						break;
					}
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
					printf( "\t%c\tSend Test HTTP request\r\n", 'm' );
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
