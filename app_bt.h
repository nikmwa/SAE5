/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef APP_BT_H_
#define APP_BT_H_

#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include <stdio.h>

#ifndef CYBSP_USER_LED2
#define CYBSP_USER_LED2 P10_0
#endif

/* Typdef for function used to free allocated buffer to stack */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/* UUIDs for service, characteristic and descriptor discovery */

#define __UUID_SERVICE_PSOC                          0x07, 0x11, 0x4A, 0xFA, 0xAA, 0x0C, 0xF1, 0x8A, 0xD7, 0x4D, 0xCC, 0x6C, 0x5C, 0x73, 0x76, 0xDB
#define __UUID_CHARACTERISTIC_PSOC_LED               0x66, 0xBD, 0x3F, 0x8A, 0x3A, 0xDE, 0xC4, 0x98, 0xE0, 0x4D, 0x14, 0x01, 0x17, 0x87, 0x08, 0x09
#define __UUID_CHARACTERISTIC_PSOC_BUTTON_COUNT      0x3D, 0xFF, 0x63, 0xF7, 0x02, 0xFB, 0x82, 0xB8, 0x69, 0x4B, 0x8D, 0x14, 0x16, 0xF3, 0x7F, 0x4F
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902

/******************************************************************************
 *                                Constants
 ******************************************************************************/
extern uint16_t bt_conn_id;
extern uint8_t ledStatus;

static const uint8_t serviceUUID[] = { __UUID_SERVICE_PSOC};
static uint16_t serviceStartHandle = 0x0001;
static uint16_t serviceEndHandle = 0xFFFF;

typedef struct {
uint16_t startHandle;
uint16_t endHandle;
uint16_t valHandle;
uint16_t cccdHandle;
} charHandle_t;

static const uint8_t ledUUID[] = { __UUID_CHARACTERISTIC_PSOC_LED };
extern charHandle_t ledChar;
static const uint8_t counterUUID[] = { __UUID_CHARACTERISTIC_PSOC_BUTTON_COUNT };
extern charHandle_t counterChar;

#define MAX_CHARS_DISCOVERED (10)
static charHandle_t charHandles[MAX_CHARS_DISCOVERED];
static uint32_t charHandleCount;

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

/* Callback function for Bluetooth stack management type events */
wiced_bt_dev_status_t app_bt_management_callback             (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* GATT Event Callback and Handler Functions */
wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);

void scanCallback													(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);

/*Discovery functions*/
void startBTServiceDiscovery(void);
void startBTCharacteristicDiscovery(void);
void startBTDescriptorDiscovery(void);

/* Helper functions to allocate/free buffers for GATT operations */
uint8_t 					*app_bt_alloc_buffer(uint16_t len);
void 					app_bt_free_buffer(uint8_t *p_data);

#endif /* APP_BT_H_ */


/* [] END OF FILE */
