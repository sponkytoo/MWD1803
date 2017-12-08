/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    fram_app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _FRAM_APP_H
#define _FRAM_APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* FRAM Command definitions */
#define FRAM_CMD_WREN               0x06
#define FRAM_CMD_WRITE              0x02
#define FRAM_CMD_READ               0x03

/* FRAM address to write temperature value to */
#define TEMPERATURE_LOG_START_ADDR         0x0000
#define MAX_TEMPERATURE_LOGS               10
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	FRAM_APP_STATE_INIT=0,
    FRAM_APP_STATE_CHECK_WRITE_REQ,
    FRAM_APP_STATE_CHECK_READ_REQ,
    FRAM_APP_STATE_CHECK_WRITE_REQ_STATUS,
    FRAM_APP_STATE_CHECK_READ_REQ_STATUS,
    FRAM_APP_STATE_ERROR,

	/* TODO: Define states used by the application state machine. */

} FRAM_APP_STATES;

typedef enum
{
    FRAM_APP_SPI_BUFFER_STATUS_COMPLETE = 0,
    FRAM_APP_SPI_BUFFER_STATUS_ERROR,
    FRAM_APP_SPI_BUFFER_STATUS_INVALID,

}FRAM_APP_SPI_BUFFER_STATUS;  

typedef struct
{
    uint8_t                             wrBuffer[50];
    uint8_t                             nBytes;
    FRAM_APP_SPI_BUFFER_STATUS          bufferStatus;
    DRV_SPI_BUFFER_HANDLE               bufferHandle;
} FRAM_APP_SPI_WR_REQ;

typedef struct
{
    uint8_t                             wrBuffer[10];
    uint8_t                             nWrBytes;
    uint8_t                             rdBuffer[50];
    uint8_t                             nRdBytes;
    FRAM_APP_SPI_BUFFER_STATUS          bufferStatus;
    DRV_SPI_BUFFER_HANDLE               bufferHandle;
} FRAM_APP_SPI_WR_RD_REQ;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    FRAM_APP_STATES         state;
    DRV_HANDLE              handle;
    /* Data structure to hold SPI request to enable FRAM writes */
    FRAM_APP_SPI_WR_REQ     framWriteEnableData;
    /* Data structure to hold SPI request to write to FRAM */
    FRAM_APP_SPI_WR_REQ     framWriteData;
    /* Data structure to hold SPI request to read from FRAM */
    FRAM_APP_SPI_WR_RD_REQ  framReadData;
    uint8_t                 keyValue;
    bool                    isWriteReq;
    uint8_t                 wrIndex;

} FRAM_APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void FRAM_APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    FRAM_APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void FRAM_APP_Initialize ( void );


/*******************************************************************************
  Function:
    void FRAM_APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    FRAM_APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void FRAM_APP_Tasks( void );

void FRAM_APP_SetWriteReq(void);

#endif /* _FRAM_APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

