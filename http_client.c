/******************************************************************************
* File Name: secure_http_client.c
*
* Description: This file contains the necessary functions to start the HTTPS
* client and send GET, POST, and PUT request to the HTTPS client.
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

/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>

/* Cypress Secure Sockets header file */
#include "cy_secure_sockets.h"
#include "cy_tls.h"

/* Wi-Fi connection manager header files */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* Standard C header file */
#include <string.h>

/* HTTPS client task header file. */
#include "http_client.h"
#include "cy_http_client_api.h"
#include "secure_keys.h"

#include "lwip/ip_addr.h"

/*******************************************************************************
* Global Variables
********************************************************************************/
bool get_after_put_flag = false;
cy_http_client_method_t http_client_method;

/* Holds the IP address obtained using Wi-Fi Connection Manager (WCM). */
static cy_wcm_ip_address_t ip_addr;

/* Secure HTTP client instance. */
cy_http_client_t http_client;

/* Holds the security configuration such as client certificate,
 * client key, and rootCA.
 */
cy_awsport_ssl_credentials_t security_config;

/* Secure HTTP server information. */
cy_awsport_server_info_t server_info;

/*Buffer to store get response*/
uint8_t http_get_buffer[HTTP_GET_BUFFER_LENGTH];

/*Holds the HTTP header fields */
cy_http_client_header_t http_header[1];

/*Holds the fields for response header and body*/
cy_http_client_response_t http_response;

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static cy_rslt_t wifi_connect(void);
void disconnect_callback_handler(cy_http_client_t handle, cy_http_client_disconn_type_t type, void *args);
static cy_rslt_t configure_http_client(cy_http_client_t* http_client);

/********************************************************************************
 * Function Name: wifi_connect
 ********************************************************************************
 * Summary:
 *  The device associates to the Access Point with given SSID, PASSWORD, and SECURITY
 *  type. It retries for MAX_WIFI_RETRY_COUNT times if the Wi-Fi connection fails.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the Wi-Fi connection is successfully
 *  established, a WCM error code otherwise.
 *
 *******************************************************************************/
cy_rslt_t wifi_connect(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint32_t retry_count = 0;
    cy_wcm_connect_params_t connect_param = {0};
    cy_wcm_config_t wcm_config = {.interface = CY_WCM_INTERFACE_TYPE_STA};


    result = cy_wcm_init(&wcm_config);



    if (CY_RSLT_SUCCESS == result)
    {


        APP_INFO(("Wi-Fi initialization is successful\n"));
        memcpy(&connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
        memcpy(&connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
        connect_param.ap_credentials.security = WIFI_SECURITY_TYPE;
        APP_INFO(("Join to AP: %s\n", connect_param.ap_credentials.SSID));


        /*
         * Connect to Access Point. It validates the connection parameters
         * and then establishes connection to AP.
         */
        for (retry_count = 0; retry_count < MAX_WIFI_RETRY_COUNT; retry_count++)
        {
             result = cy_wcm_connect_ap(&connect_param, &ip_addr);

             if (CY_RSLT_SUCCESS == result)
             {
                 APP_INFO(("Successfully joined Wi-Fi network %s\n", connect_param.ap_credentials.SSID));

                 if (CY_WCM_IP_VER_V4 == ip_addr.version)
                 {
                     APP_INFO(("Assigned IP address: %s\n", ip4addr_ntoa((const ip4_addr_t *)&ip_addr.ip.v4)));
                 }
                 else if (CY_WCM_IP_VER_V6 == ip_addr.version)
                 {
                     APP_INFO(("Assigned IP address: %s\n", ip6addr_ntoa((const ip6_addr_t *)&ip_addr.ip.v6)));
                 }

                 break;
             }

             ERR_INFO(("Failed to join Wi-Fi network. Retrying...\n"));
        }


    }

    return result;
}

/*******************************************************************************
 * Function Name: disconnect_callback
 *******************************************************************************
 * Summary:
 *  Callback function for http disconnect
 *
 * Parameters:
 *  void
 *
 * Return:
 *  None.
 *
 *******************************************************************************/
void disconnect_callback_handler(cy_http_client_t handle, cy_http_client_disconn_type_t type, void *args)
{
    printf("\nApplication Disconnect callback triggered for handle = %p type=%d\n", handle, type);
}

/*******************************************************************************
 * Function Name: configure_http_client
 *******************************************************************************
 * Summary:
 *  Configures the security parameters such as client certificate, private key,
 *  and the root CA certificate to start the HTTP client in secure mode.
 *
 * Parameters:
 *  cy_http_client_t* http_client
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the secure HTTP client is configured
 *  successfully, otherwise, it returns CY_RSLT_TYPE_ERROR.
 *
 *******************************************************************************/
static cy_rslt_t configure_http_client(cy_http_client_t* http_client)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_http_disconnect_callback_t http_cb;



    server_info.host_name = HTTP_SERVER_HOST;
    server_info.port = HTTP_PORT;

    /* Initialize the HTTP Client Library. */
    result = cy_http_client_init();
    if( result != CY_RSLT_SUCCESS )
    {
        /* Failure path. */
        ERR_INFO(("Failed to initialize http client.\n"));
    }



    http_cb = disconnect_callback_handler;



    /* Create an instance of the HTTP client. */
    result = cy_http_client_create(NULL, &server_info, http_cb, NULL, http_client);


    
    if( result != CY_RSLT_SUCCESS )
    {
        /* Failure path */
        ERR_INFO(("Failed to create http client.\n"));
    }
    return result;
}

/*******************************************************************************
 * Function Name: http_client_task
 *******************************************************************************
 * Summary:
 *  Starts the HTTP client in secure mode. This example application is using a
 *  self-signed certificate which means there is no third-party certificate issuing
 *  authority involved in the authentication of the client. It is the user's
 *  responsibility to supply the necessary security configurations such as client's
 *  certificate, private key of the client, and RootCA of the client to start the
 *  HTTP client in secure mode.
 *
 * Parameters:
 *  void * pvParameters 
 *
 * Return:
 *  None.
 *
 *******************************************************************************/
void http_client_task( void * pvParameters )
{
    cy_rslt_t result = CY_RSLT_TYPE_ERROR;

    cy_http_client_t *HTTPclient;
    HTTPclient = (cy_http_client_t *) pvParameters;

    /* Connects to the Wi-Fi Access Point. */
    result = wifi_connect();
    PRINT_AND_ASSERT(result, "Wi-Fi connection failed.\n");

    /* Configure the HTTPS client and
     * register a default dynamic URL handler.
     */
    result = configure_http_client(HTTPclient);
    PRINT_AND_ASSERT(result, "Failed to configure the HTTPS client.\n");

    /* Connect the HTTP client to server. */
    result = cy_http_client_connect(*HTTPclient, TRANSPORT_SEND_RECV_TIMEOUT_MS, TRANSPORT_SEND_RECV_TIMEOUT_MS);
    if( result != CY_RSLT_SUCCESS )
    {
        ERR_INFO(("Failed to connect to the http server.\n"));
    }
    else
    {
        printf("Successfully connected to http server\r\n");
        while(true)
        {
            // nothing
        }
    }
}


/*******************************************************************************
 * Function Name: send_http_example_request
 *******************************************************************************
 * Summary:
 *  The function handles an http send POST operation to the server.
 *  Sent data: temperature: 25+rand()%10
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the secure HTTP client is configured
 *  successfully, otherwise, it returns CY_RSLT_TYPE_ERROR.
 *
 *******************************************************************************/
cy_rslt_t send_http_example_request( cy_http_client_t handle, cy_http_client_method_t method,
                               const char * pPath)
{
    cy_http_client_request_header_t request;
    cy_http_client_header_t header;

    cy_http_client_response_t response;

    /* Return value of all methods from the HTTP Client library API. */
    cy_rslt_t http_status = CY_RSLT_SUCCESS;

    /* Initialize the response object. The same buffer used for storing
     * request headers is reused here. */
    request.buffer = http_get_buffer;
    request.buffer_len = HTTP_GET_BUFFER_LENGTH;

    request.headers_len = HTTP_REQUEST_HEADER_LEN;
    request.method = method;
    request.range_end = HTTP_REQUEST_RANGE_END;
    request.range_start = HTTP_REQUEST_RANGE_START;
    request.resource_path = pPath;

    header.field = "Content-Type";
    header.field_len = sizeof("Content-Type")-1;
    header.value = "application/json";
    header.value_len = sizeof("application/json")-1;

    http_status = cy_http_client_write_header(handle, &request, &header, NUM_HTTP_HEADERS);
    if( http_status != CY_RSLT_SUCCESS )
    {
        printf("\nWrite Header ----------- Fail \n");
        return http_status;
    }
    else
    {
        printf( "\n Sending Request Headers:\n%.*s\n",( int ) request.headers_len, ( char * ) request.buffer);
    }

    uint8_t body[22];
    snprintf(body,22,"{\"temperature\": %d}", 25 + rand() % 20);
    printf("\n size of:%d\n",sizeof("{\"temperature\": 32}")-1);
    //http_status = cy_http_client_send(handle, &request, "{\"temperature\": 32}", sizeof("{\"temperature\": 32}")-1, &response);
    http_status = cy_http_client_send(handle, &request, body, 19, &response);
    if( http_status != CY_RSLT_SUCCESS )
    {
        printf("\nFailed to send HTTP method=%d\n Error=%ld\r\n",request.method,(unsigned long)http_status);
        return http_status;
    }
    else
    {
        if ( CY_HTTP_CLIENT_METHOD_HEAD != method )
        {
            TEST_INFO(( "Received HTTP response from %.*s%.*s...\n"
                   "Response Headers:\n %.*s\n"
                   "Response Status :\n %u \n"
                   "Response Body   :\n %.*s\n",
                   ( int ) sizeof(HTTP_SERVER_HOST)-1, HTTP_SERVER_HOST,
                   ( int ) sizeof(request.resource_path) -1, request.resource_path,
                   ( int ) response.headers_len, response.header,
                   response.status_code,
                   ( int ) response.body_len, response.body ) );

        }
        printf("\n buffer_len:[%d] headers_len:[%d] header_count:[%d] body_len:[%d] content_len:[%d]\n",
                 response.buffer_len, response.headers_len, response.header_count, response.body_len, response.content_len);
    }

    return http_status;
}

/*******************************************************************************
 * Function Name: send_http_counter_request
 *******************************************************************************
 * Summary:
 *  The function handles an http send POST operation to the server.
 *  Sent data: counter: count
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t: Returns CY_RSLT_SUCCESS if the secure HTTP client is configured
 *  successfully, otherwise, it returns CY_RSLT_TYPE_ERROR.
 *
 *******************************************************************************/
cy_rslt_t send_http_counter_request( cy_http_client_t handle, cy_http_client_method_t method,
                               const char * pPath, int16_t count)
{
    cy_http_client_request_header_t request;
    cy_http_client_header_t header;

    cy_http_client_response_t response;

    /* Return value of all methods from the HTTP Client library API. */
    cy_rslt_t http_status = CY_RSLT_SUCCESS;

    /* Initialize the response object. The same buffer used for storing
     * request headers is reused here. */
    request.buffer = http_get_buffer;
    request.buffer_len = HTTP_GET_BUFFER_LENGTH;

    request.headers_len = HTTP_REQUEST_HEADER_LEN;
    request.method = method;
    request.range_end = HTTP_REQUEST_RANGE_END;
    request.range_start = HTTP_REQUEST_RANGE_START;
    request.resource_path = pPath;

    header.field = "Content-Type";
    header.field_len = sizeof("Content-Type")-1;
    header.value = "application/json";
    header.value_len = sizeof("application/json")-1;

    http_status = cy_http_client_write_header(handle, &request, &header, NUM_HTTP_HEADERS);
    if( http_status != CY_RSLT_SUCCESS )
    {
        printf("\nWrite Header ----------- Fail \n");
        return http_status;
    }
    else
    {
        printf( "\n Sending Request Headers:\n%.*s\n",( int ) request.headers_len, ( char * ) request.buffer);
    }

    uint8_t body[22];
    snprintf(body, 22, "{\"counter\": %d}", count);
    uint8_t length = strlen(body);

    http_status = cy_http_client_send(handle, &request, body, length, &response);
    if( http_status != CY_RSLT_SUCCESS )
    {
        printf("\nFailed to send HTTP method=%d\n Error=%ld\r\n",request.method,(unsigned long)http_status);
        return http_status;
    }
    else
    {
        if ( CY_HTTP_CLIENT_METHOD_HEAD != method )
        {
            TEST_INFO(( "Received HTTP response from %.*s%.*s...\n"
                   "Response Headers:\n %.*s\n"
                   "Response Status :\n %u \n"
                   "Response Body   :\n %.*s\n",
                   ( int ) sizeof(HTTP_SERVER_HOST)-1, HTTP_SERVER_HOST,
                   ( int ) sizeof(request.resource_path) -1, request.resource_path,
                   ( int ) response.headers_len, response.header,
                   response.status_code,
                   ( int ) response.body_len, response.body ) );

        }
        printf("\n buffer_len:[%d] headers_len:[%d] header_count:[%d] body_len:[%d] content_len:[%d]\n",
                 response.buffer_len, response.headers_len, response.header_count, response.body_len, response.content_len);
    }

    return http_status;
}

/* [] END OF FILE */

