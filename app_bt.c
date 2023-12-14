#include "app_bt.h"

/* FreeRTOS header file */
#include "FreeRTOS.h"
#include <task.h>

#include "cy_wcm.h"
#include "cybsp.h"
#include "cy_network_mw_core.h"
#include "cyhal_gpio.h"
#include <stdio.h>

/* Bluetooth */
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "cycfg_gap.h"


/* Library for malloc and free */
#include "stdlib.h"

/* HTTP client */
#include "http_client.h"
#include "cy_http_client_api.h"

/******************************************************
 *               Function prototypes
 ******************************************************/

/* GATT Event Callback and Handler Functions */
wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);


/******************************************************
 *                   Variables
 ******************************************************/
uint16_t bt_conn_id;
uint8_t ledStatus[];

charHandle_t ledChar;
charHandle_t counterChar;

/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
    {
	/* Start in error state so that any unimplemented states will return error */
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_device_address_t bda = {0};

    printf("Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				printf( "Bluetooth Enabled\n" );

				/* Set the local BDA from the value in the configurator and print it */
				wiced_bt_set_local_bdaddr((uint8_t*)cy_bt_device_address, BLE_ADDR_PUBLIC);
				wiced_bt_dev_read_local_addr( bda );
				printf( "Local Bluetooth Device Address: ");
				print_bd_address(bda);

				/* Register GATT callback */
				wiced_bt_gatt_register( app_bt_gatt_event_callback );

	            result = WICED_BT_SUCCESS;
    }
    else
    {
				printf( "Failed to initialize Bluetooth controller and stack\n" );
    }
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
			p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
			p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRING_COMPLETE_EVT:
			printf("Pairing complete: %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.status);
			result = WICED_BT_SUCCESS;
			break;

		case BTM_ENCRYPTION_STATUS_EVT:
			printf("Encrypt status: %d\n", p_event_data->encryption_status.result);
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
			result = WICED_BT_SUCCESS;
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
			result = WICED_BT_ERROR; // Return error since keys are not stored in EEPROM
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			result = WICED_BT_SUCCESS;
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 				// Read keys from NVRAM
            /* This should return WICED_BT_SUCCESS if not using privacy. If RPA is enabled but keys are not
               stored in EEPROM, this must return WICED_BT_ERROR so that the stack will generate new privacy keys */
			result = WICED_BT_ERROR;
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			switch( p_event_data->ble_scan_state_changed )
    {
				case BTM_BLE_SCAN_TYPE_NONE:
					printf( "Scanning stopped.\r\n" );
					cyhal_gpio_write(CYBSP_USER_LED,CYBSP_LED_STATE_OFF);
					break;

				case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
					printf( "High duty scanning.\r\n" );
					cyhal_gpio_write(CYBSP_USER_LED,CYBSP_LED_STATE_ON);
					break;

				case BTM_BLE_SCAN_TYPE_LOW_DUTY:
					printf( "Low duty scanning.\r\n" );
					cyhal_gpio_write(CYBSP_USER_LED,CYBSP_LED_STATE_ON);
					break;
    }
			result = WICED_BT_SUCCESS;
			break;

		case BTM_BLE_CONNECTION_PARAM_UPDATE:
			result = WICED_BT_SUCCESS;
			break;

		default:
			break;
    }

    return result;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_event_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_event_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	/* Start in error state so that any unimplemented states will return error */
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
     * parameters to the callback function */
    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = app_bt_connect_event_handler(&p_event_data->connection_status);
        break;

    case GATT_OPERATION_CPLT_EVT:
     	/* Look for any type of successful GATT completion */
    	if (p_event_data->operation_complete.status == WICED_BT_GATT_SUCCESS ||
			p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPTED_MITM ||
    		p_event_data->operation_complete.status == WICED_BT_GATT_ENCRYPTED_NO_MITM ||
			p_event_data->operation_complete.status == WICED_BT_GATT_NOT_ENCRYPTED)
    	{
    		printf("GATT operation completed successfully\n");
			status = WICED_BT_GATT_SUCCESS;
			if ( GATTC_OPTYPE_READ_HANDLE == p_event_data->operation_complete.op )
			{
    			if(p_event_data->operation_complete.response_data.handle == ledChar.valHandle)
    			{
    				printf("LED value is: %d\n",ledStatus);
    			}
			} else if ( GATTC_OPTYPE_NOTIFICATION == p_event_data->operation_complete.op )
			{
				if(p_event_data->operation_complete.response_data.handle == counterChar.valHandle)
    			{
    				printf("Count notification received: %d\n", *p_event_data->operation_complete.response_data.att_value.p_data);
    			    // send_http_counter_request(http_client,CY_HTTP_CLIENT_METHOD_POST,HTTP_PATH,*p_event_data->operation_complete.response_data.att_value.p_data); 
                }
			}
    	}
    	else
    	{
    		printf("GATT operation failed with status: %d\n", p_event_data->operation_complete.status);
    	}
    	break;

    case GATT_GET_RESPONSE_BUFFER_EVT: /* GATT buffer request, typically sized to max of bearer mtu - 1 */
    	p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT: /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
            {
                pfn_free(p_event_data->buffer_xmitted.p_app_data);
            }
            status = WICED_BT_GATT_SUCCESS;
        }
        break;
	case GATT_DISCOVERY_RESULT_EVT:
		{
		//////////////// Services Discovery /////////////////
		if(GATT_DISCOVER_SERVICES_BY_UUID == p_event_data->discovery_result.discovery_type)
		{
			serviceStartHandle = p_event_data->discovery_result.discovery_data.group_value.s_handle;
			serviceEndHandle = p_event_data->discovery_result.discovery_data.group_value.e_handle;
			printf( "Discovered Service Start=0x%04X End=0x%04X\r\n", serviceStartHandle, serviceEndHandle );
		} 

		//////////////// Characteristics Discovery /////////////////
		if (GATT_DISCOVER_CHARACTERISTICS == p_event_data->discovery_result.discovery_type)
		{
			charHandles[charHandleCount].startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
			charHandles[charHandleCount].valHandle =   p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
			charHandles[charHandleCount].endHandle = serviceEndHandle; /* Assume this is the last characteristic in the service so its end handle is at the end of the service group */
			
			printf( "Char Handle=0x%04X Value Handle=0x%04X ", charHandles[charHandleCount].startHandle, charHandles[charHandleCount].valHandle);
			
			if( charHandleCount != 0 )
			{
				charHandles[charHandleCount-1].endHandle =	charHandles[charHandleCount].startHandle-1;
			}
			charHandleCount += 1;

			if( charHandleCount > MAX_CHARS_DISCOVERED-1 )
			{
				printf( "This is really bad.. we discovered more characteristics than we can save\r\n" );
			}

			/*Look for 2 byte UUIDs */
			if ( p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.len == LEN_UUID_16)
			{
				if( memcmp( ledUUID, p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128, LEN_UUID_16 ) == 0 ) // If it is the LED characteristic
				{
                    ledChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
					ledChar.valHandle =   p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
					printf( "   LED   ");
				}

				if( memcmp( counterUUID, p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128,LEN_UUID_16 ) == 0 ) // If it is the button count characteristic
				{
                    counterChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
					counterChar.valHandle =  p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
					printf( "   BTN   ");
				}

				printf( "UUID: ");
				for (int i=0; i < p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.len; i++ ) // Dump the UUID bytes to the screen
				{
					printf( "%02X ", p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128[i] );
				}
			}


			/* Look only for 16 byte UUIDs */
			if( p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.len == LEN_UUID_128)
			{
				if( memcmp( ledUUID, p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128, LEN_UUID_128 ) == 0 ) // If it is the LED characteristic
				{
                    ledChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
					ledChar.valHandle =   p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
				}

				if( memcmp( counterUUID, p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128,LEN_UUID_128 ) == 0 ) // If it is the button count characteristic
				{
                    counterChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
					counterChar.valHandle =  p_event_data->discovery_result.discovery_data.characteristic_declaration.val_handle;
				}

				printf( "UUID: ");
				for (int i=0; i < p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.len; i++ ) // Dump the UUID bytes to the screen
				{
					printf( "%02X ", p_event_data->discovery_result.discovery_data.characteristic_declaration.char_uuid.uu.uuid128[i] );
				}
			}
			printf( "\r\n" );
		}
		//////////////// Characteristics Discovery /////////////////
		if (GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS == p_event_data->discovery_result.discovery_type)
		{
			if ( p_event_data->discovery_result.discovery_data.char_descr_info.type.uu.uuid16 == __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION )
			{
				counterChar.cccdHandle = p_event_data->discovery_result.discovery_data.char_descr_info.handle;
				
				/* Print out the handle and UUID of the CCCD */
				printf( "Char Descriptor Handle = 0x%04X, UUID: ", p_event_data->discovery_result.discovery_data.char_descr_info.handle);

				for( int i=0; i<p_event_data->discovery_result.discovery_data.char_descr_info.type.len; i++ )
				{
					/* We will use the uuid128 value from the union and just print out as many bytes as the len parameter
					 * indicates. This allows us to print any type of UUID */
					printf( "%02X ", p_event_data->discovery_result.discovery_data.char_descr_info.type.uu.uuid128[i] );
				}
				printf( "\r\n" );
			}
		}
		}

		break;
	case GATT_DISCOVERY_CPLT_EVT:
    	/* Once all characteristics are discovered... you need to setup the end handles */
    	if( p_event_data->discovery_complete.discovery_type == GATT_DISCOVER_CHARACTERISTICS )
    	{
    	  for( int i=0; i<charHandleCount; i++ )
    	  {
    	    if( charHandles[i].startHandle == ledChar.startHandle )
    	      ledChar.endHandle = charHandles[i].endHandle;
    	    if( charHandles[i].startHandle == counterChar.startHandle )
    	      counterChar.endHandle = charHandles[i].endHandle;
    	  }
    	}

    	break;
    default:
    	printf( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}


/*******************************************************************************
 * Function Name: app_bt_connect_event_handler
 *
 * Handles GATT connection status changes.
 *
 * Param:	p_conn_status  Pointer to data that has connection details
 * Return:	wiced_bt_gatt_status_t
 * See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*******************************************************************************/
wiced_bt_gatt_status_t app_bt_connect_event_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
           	/* Handle the connection */
            printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
           	print_bd_address(p_conn_status->bd_addr);
			printf("Connection ID %d\n", p_conn_status->conn_id );
            bt_conn_id = p_conn_status->conn_id;

			cyhal_gpio_write(CYBSP_USER_LED,CYBSP_LED_STATE_ON);

			/* Initiate pairing */
			wiced_bt_dev_sec_bond(
							p_conn_status->bd_addr,
							p_conn_status->addr_type,
							BT_TRANSPORT_LE,
							0,
							NULL
							);
        }
        else
        {
            /* Handle the disconnection */
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );
			bt_conn_id = 0;

			cyhal_gpio_write(CYBSP_USER_LED,CYBSP_LED_STATE_OFF);
        }

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*******************************************************************************
* Function Name: BLEScanCallback(p_scan_result, *p_adv_data)
********************************************************************************/
void BLEScanCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
	#define MAX_ADV_NAME_LEN	(28) 		/* Maximum possible name length since flags take 3 bytes and max packet is 31. */
	#define SEARCH_DEVICE_NAME	"aly_per"	/* Name of device to search for */

	uint8_t length;
	uint8_t *p_name = NULL;
	uint8_t dev_name[MAX_ADV_NAME_LEN];

	p_name = wiced_bt_ble_check_advertising_data(p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &length);
	if ( p_name && (length == strlen(SEARCH_DEVICE_NAME)) && (memcmp(SEARCH_DEVICE_NAME, p_name, length)==0) )
	{
		memcpy(dev_name, p_name, length);
		dev_name[length] = 0x00;	/* Null terminate the string */

		printf("Found Device \"%s\" with BD Address", dev_name);
		print_bd_address(p_scan_result->remote_bd_addr);

		/* Connect to peripheral and stop scanning*/
		wiced_bt_gatt_le_connect(
			p_scan_result->remote_bd_addr,
			p_scan_result->ble_addr_type,
			BLE_CONN_MODE_HIGH_DUTY,
			WICED_TRUE);
		wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, BLEScanCallback );
	}
}

/*******************************************************************************
* Function Name: void writeAttribute
********************************************************************************/
void writeAttribute( uint16_t bt_conn_id, uint16_t handle, uint16_t offset, wiced_bt_gatt_auth_req_t auth_req, uint16_t len, uint8_t* val )
{
	if  (bt_conn_id && handle ) 
	{
		wiced_bt_gatt_write_hdr_t write_params;
		write_params.handle = handle;
		write_params.offset = offset;
		write_params.auth_req = auth_req;
		write_params.len = len;

		wiced_bt_gatt_client_send_write(bt_conn_id, GATT_REQ_WRITE, &write_params, val, NULL);
	}
}

void startBTServiceDiscovery( void )
{
	wiced_bt_gatt_discovery_param_t discovery_param;
	memset( &discovery_param, 0, sizeof( discovery_param ) );
	discovery_param.s_handle = 0x0001;
	discovery_param.e_handle = 0xFFFF;
	discovery_param.uuid.len = LEN_UUID_16;
	memcpy( &discovery_param.uuid.uu.uuid16, serviceUUID, LEN_UUID_16 );

	wiced_bt_gatt_status_t status = wiced_bt_gatt_client_send_discover(bt_conn_id,GATT_DISCOVER_SERVICES_BY_UUID, &discovery_param);
	printf( "Started service discovery. Status: 0x%02X\r\n", status );
}

void startBTCharacteristicDiscovery( void )
{
	charHandleCount = 0;

	wiced_bt_gatt_discovery_param_t discovery_param;
	discovery_param.s_handle = serviceStartHandle+1;
	discovery_param.e_handle = serviceEndHandle;

	wiced_bt_gatt_status_t status = wiced_bt_gatt_client_send_discover(bt_conn_id,GATT_DISCOVER_CHARACTERISTICS, &discovery_param);
	printf( "Started characteristic discovery. Status: 0x%02X\r\n", status );
}

void startBTDescriptorDiscovery( void )
{
	wiced_bt_gatt_discovery_param_t discovery_param;
	discovery_param.s_handle = counterChar.startHandle+1;
	discovery_param.e_handle = counterChar.endHandle;

	wiced_bt_gatt_status_t status = wiced_bt_gatt_client_send_discover(bt_conn_id,GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &discovery_param);
	printf( "Started descriptor discovery. Status: 0x%02X\r\n", status );
}


/*******************************************************************************
* Function Name: app_bt_alloc_buffer
*
* This Function allocates the buffer of requested length
*
* Param:  len			Length of buffer
* Return: uint8_t*      Pointer to allocated buffer
********************************************************************************/
uint8_t *app_bt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    return p;
}


/*******************************************************************************
* Function Name: app_bt_free_buffer
*
* This Function frees the buffer requested
*
* Param:  p_data		Pointer to buffer to be freed
********************************************************************************/
void app_bt_free_buffer(uint8_t *p_data)
{
    if (p_data != NULL)
    {
        free(p_data);
    }
}
