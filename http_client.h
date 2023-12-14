/******************************************************************************
* File Name: secure_http_client.h
*
* Description: This file contains configuration parameters for the secure HTTP
* client along with the HTML page that the client will host.
*
* Related Document: See README.md
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
* Include guard
*******************************************************************************/
#ifndef HTTP_CLIENT_H_
#define HTTP_CLIENT_H_

/* FreeRTOS header file */
#include "FreeRTOS.h"
#include <task.h>
#include "cy_wcm.h"
#include "cybsp.h"
#include "cy_network_mw_core.h"
#include "cyhal_gpio.h"
#include "cy_http_client_api.h"

#define TEST_INFO( x )                        printf x

/* Wi-Fi Credentials: Modify WIFI_SSID and WIFI_PASSWORD to match your Wi-Fi network
 * Credentials.
 */
#define WIFI_SSID                                "NETGEAR0-24G"
#define WIFI_PASSWORD                            "departementGEII"

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY_TYPE                       CY_WCM_SECURITY_WPA2_AES_PSK

#define MAX_WIFI_RETRY_COUNT                     (3)

#define APP_INFO(x)                              do { printf("Info: "); printf x; } while(0);
#define ERR_INFO(x)                              do { printf("Error: "); printf x; } while(0);

#define PRINT_AND_ASSERT(result, msg, args...)   do                                 \
                                                 {                                  \
                                                     if (CY_RSLT_SUCCESS != result) \
                                                     {                              \
                                                         ERR_INFO((msg, ## args));  \
                                                         CY_ASSERT(0);              \
                                                     }                              \
                                                 } while(0);

#define HTTP_PORT                               (8080)
#define HTTP_SERVER_HOST                        "192.168.0.2"
#define TRANSPORT_SEND_RECV_TIMEOUT_MS           (5000)
#define HTTP_GET_BUFFER_LENGTH                   (2048)
#define ASCII_INTEGER_DIFFERENCE                 (48)
#define HTTP_PATH                                "/api/v1/kzbNRI2CnlfaBbWyOb84/telemetry"

/* Wi-Fi re-connection time interval in milliseconds */
#define WIFI_CONN_RETRY_INTERVAL_MSEC            (1000u)

/*End Range until where the data is expected. Set Set this to -1 if requested range is
 * all bytes from the starting*/
#define HTTP_REQUEST_RANGE_END                   (-1)

/*Start Range from where the server should return.*/
#define HTTP_REQUEST_RANGE_START                 (0)

/*Number of headers in the header list*/
#define NUM_HTTP_HEADERS                         (1)

/*Length of the request header.*/
#define HTTP_REQUEST_HEADER_LEN                  (0)

/******************************************************
 *                   Variable
 ******************************************************/
extern cy_http_client_t http_client;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void http_client_task(void *arg);
cy_rslt_t send_http_example_request(cy_http_client_t handle,cy_http_client_method_t method,const char * pPath);
cy_rslt_t send_http_counter_request(cy_http_client_t handle,cy_http_client_method_t method,const char * pPath, int16_t counter);

#endif /* HTTP_CLIENT_H_ */


/* [] END OF FILE */
