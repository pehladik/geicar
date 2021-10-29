/***************************************************************************************************
**    Copyright (C) 2018 - 2019 HMS Technology Center Ravensburg GmbH, all rights reserved
****************************************************************************************************
**
**        File: simply.h
**     Summary: SimplyCAN API for C
**              The simplyCAN API provides a simple programming interface for the development
**              of CAN applications on Windows PCs and on Linux PCs.
**
****************************************************************************************************
**    This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY.
***************************************************************************************************/

#ifndef SIMPLY_H
#define SIMPLY_H

/* Definition of calling attribute (DLL) */
#if defined(_WIN32) || defined(WIN32)
#define DLL __declspec(dllexport)
#else
#define DLL
#endif

/***************************************************************************************************
**    constants and macros
***************************************************************************************************/
/* simplyCAN API version */
#define SIMPLY_API_VERSION_MAJOR              1
#define SIMPLY_API_VERSION_MINOR              0
#define SIMPLY_API_VERSION_BUILD              0

/* CAN status definitions */
#define CAN_STATUS_RUNNING             (0x01)
#define CAN_STATUS_RESET               (0x02)
#define CAN_STATUS_BUSOFF              (0x04)
#define CAN_STATUS_ERRORSTATUS         (0x08)
#define CAN_STATUS_RXOVERRUN           (0x10)
#define CAN_STATUS_TXOVERRUN           (0x20)
#define CAN_STATUS_PENDING             (0x40)

/***************************************************************************************************
**    data types
***************************************************************************************************/
/** simply_last_error_t:
    Error codes of the API functions.
    - Return value code 0 indicates that no error occured.
    - Actual errors have a negative value code.
    - Non-critical errors have a positive value code.
*/
typedef enum _simply_last_error {
	SIMPLY_S_NO_ERROR = 0,      /*  No error occurred */
	SIMPLY_E_SERIAL_OPEN = -1,      /*  Unable to open the serial port */
	SIMPLY_E_SERIAL_ACCESS = -2,      /*  Access on serial port denied */
	SIMPLY_E_SERIAL_CLOSED = -3,      /*  Serial communication port is closed */
	SIMPLY_E_SERIAL_COMM = -4,      /*  Serial communication error */
	SIMPLY_E_CMND_REQ_UNKNOWN = -5,      /*  Command unknown on device */
	SIMPLY_E_CMND_RESP_TIMEOUT = -6,      /*  Command response timeout reached */
	SIMPLY_E_CMND_RESP_UNEXPECTED = -7,      /*  Unexpected command response received */
	SIMPLY_E_CMND_RESP_ERROR = -8,      /*  Command response error */
	SIMPLY_E_INVALID_PROTOCOL_VERSION = -9,      /*  Invalid simplyCAN protocol version */
	SIMPLY_E_INVALID_FW_VERSION = -10,     /*  Invalid device firmware version */
	SIMPLY_E_INVALID_PRODUCT_STRING = -11,     /*  Invalid simplyCAN product string */
	SIMPLY_E_CAN_INVALID_STATE = -12,     /*  Invalid CAN state */
	SIMPLY_E_CAN_INVALID_BAUDRATE = -13,     /*  Invalid CAN baud-rate */
	SIMPLY_E_TX_BUSY = -14,     /*  Message could not be sent. TX is busy */
	SIMPLY_E_API_BUSY = -15,     /*  API is busy */
} simply_last_error_t;

// Device identification
typedef struct _identification {
	uint8_t fw_version[8];              // Zero terminated firmware version string e.g. "1.00.00"
	uint8_t hw_version[8];              // Zero terminated hardware version string e.g. "1.00.00"
	uint8_t product_version[8];         // Zero terminated product version string e.g. "1.00.00"
	uint8_t product_string[30];         // Zero terminated product string e.g. "simplyCAN 1.0"
	uint8_t serial_number[9];           // Zero terminated serial number e.g. "HW123456"
} identification_t;

// CAN Message
typedef struct _can_msg {
	uint32_t timestamp;               // in milliseconds
	uint32_t ident;                   // MSB=1: extended frame
	uint8_t dlc;                      // MSB=1: remote frame
	uint8_t payload[8];
} can_msg_t;

// CAN Status
typedef struct _can_sts {
	uint16_t sts;                      // bit coded status flags (see CAN status definitions)
	uint16_t tx_free;                  // number of free elements in CAN message tx FIFO
} can_sts_t;

/***************************************************************************************************
**    global variables
***************************************************************************************************/

/***************************************************************************************************
**    function prototypes
***************************************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************
  Function:
    simply_open

  Description:
    Opens the serial communication interface. The message filter of the CAN controller is opened
    for all message identifiers.

  Parameters:
    serial_port   (IN) - Name of the serial communication port (e.g. COM1 or /dev/ttyACM0).
                         Use the simplyCAN bus monitor to detect on which serial COM port the 
                         simplyCAN is connected. With Windows it is also possible to use the 
                         device manager and with Linux the command "ls -l /dev/serial/by-id".

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_open(char *serial_port);

/***************************************************************************************************
  Function:
    simply_close

  Description:
    Closes the serial communication interface and resets the CAN controller.

  Parameters:
    -

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_close(void);

/***************************************************************************************************
  Function:
    simply_initialize_can

  Description:
    Initializes the CAN controller.

  Parameters:
    bitrate       (IN) - CAN bitrate as integer value,
                         possible values: 10, 20, 50, 125, 250, 500, 800, 1000

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_initialize_can(uint16_t bitrate);

/***************************************************************************************************
  Function:
    simply_identify

  Description:
    Gets firmware and hardware information about the simplyCAN device.

  Parameters:
    p_identification   (OUT) - Pointer to the identification structure

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_identify(identification_t *p_identification);

/***************************************************************************************************
  Function:
    simply_start_can

  Description:
    Starts the CAN controller. Sets the CAN controller into running mode and clears the CAN
    message FIFOs. In running mode CAN messages can be transmitted and received.

  Parameters:
    -

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_start_can(void);

/***************************************************************************************************
  Function:
    simply_stop_can

  Description:
    Stops the CAN controller. Sets the CAN controller into init mode. Does not reset the message
    filter of the CAN controller. Only stop the CAN controller when the CAN_STATUS_PENDING flag
    is not set.

  Parameters:
    -

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_stop_can(void);

/***************************************************************************************************
  Function:
    simply_reset_can

  Description:
    Resets the CAN controller (hardware reset) and clears the message filter (open for all message
    identifiers). Sets the CAN controller into init mode.

  Parameters:
    -

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_reset_can(void);

/***************************************************************************************************
  Function:
    simply_can_status

  Description:
    Gets the status of the CAN controller.

  Parameters:
    can_sts       (OUT) - status as bit coded 16 bit value (see can_sts_t)

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_can_status(can_sts_t *can_sts);

/***************************************************************************************************
  Function:
    simply_set_filter

  Description:
    Sets the 11 or 29 bit message filter of the CAN controller. To set the 29 bit message filter,
    the MSB in parameter value must be set.

  Parameters:
    mask          (IN) - 11 or 29 bit CAN message identifier mask
    value         (IN) - 11 or 29 bit CAN message identifier value, set MSB to set the 29 bit 
                         message filter

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_set_filter(uint32_t mask, uint32_t value);

/***************************************************************************************************
  Function:
    simply_receive

  Description:
    Receives a single CAN message.

  Parameters:
    can_msg       (OUT) - Pointer to the CAN message structure the received CAN message is
                          copied to

  Return value:
    1                  Message received
    0                  No message available in the receive queue
    -1                 Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL int8_t simply_receive(can_msg_t *can_msg);

/***************************************************************************************************
  Function:
    simply_send

  Description:
    Writes a CAN message to the transmit FIFO. To check if the message is transmitted, request
    the CAN status with simply_can_status.

  Parameters:
    can_msg       (IN) - Pointer to a CAN message to be transmitted (see can_msg_t)

  Return value:
    true               Function succeeded
    false              Error occurred, call simply_get_last_error for more information.

***************************************************************************************************/
DLL bool simply_send(can_msg_t *can_msg);

/***************************************************************************************************
  Function:
    simply_get_last_error

  Description:
    Gets the last error code. After reading the error code with simply_get_last_error the error
    code is set to 0. Each error can only be read once.

  Parameters:
    -

  Return value:
    0                  No error occurred
    <0                 Error occurred (see simply_last_error_t)

***************************************************************************************************/
DLL int16_t simply_get_last_error(void);

#ifdef __cplusplus
};
#endif // __cplusplus

#endif // SIMPLY_H
