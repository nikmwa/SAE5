/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef APP_BT_H_
#define APP_BT_H_

#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include <stdio.h>

#ifndef CYBSP_USER_LED
#define CYBSP_USER_LED P10_0
#endif

/* Typdef for function used to free allocated buffer to stack */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/* UUIDs for service, characteristic and descriptor discovery */

#define __UUID_SERVICE_PSOC                          0x00, 0xFF
#define __UUID_CHARACTERISTIC_PSOC_LED               0x04, 0xFF
#define __UUID_CHARACTERISTIC_PSOC_BUTTON_COUNT      0x03, 0xFF
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902

/******************************************************************************
 *                                Constants
 ******************************************************************************/
extern uint16_t bt_conn_id;
extern uint8_t ledStatus[];

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

#define MAX_CHARS_DISCOVERED (20)
static charHandle_t charHandles[MAX_CHARS_DISCOVERED];
static uint32_t charHandleCount;

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

/* Callback function for Bluetooth stack management type events */
wiced_bt_dev_status_t app_bt_management_callback    (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/* Callback function for BLE scanning */
void BLEScanCallback			                    (wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);

/*Discovery functions*/
void startBTServiceDiscovery                        (void);
void startBTCharacteristicDiscovery                 (void);
void startBTDescriptorDiscovery                     (void);

/* Helper functions to allocate/free buffers for GATT operations */
uint8_t *app_bt_alloc_buffer                        (uint16_t len);
void app_bt_free_buffer                             (uint8_t *p_data);

#endif /* APP_BT_H_ */


/* [] END OF FILE */
