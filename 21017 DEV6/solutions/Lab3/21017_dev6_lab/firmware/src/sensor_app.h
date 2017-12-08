/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    sensor_app.h

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

#ifndef _SENSOR_APP_H
#define _SENSOR_APP_H

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

#define TEMPERATURE_READ_PERIOD                                     500

#define WR(addr)                                                    (addr & 0x7F)
#define RD(addr)                                                    (addr | 0x80)
#define BME280_RST_REG                                              0xE0  /*Softreset Register */
#define BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG                     0x88  /*Temperature Calibration Start Register*/
#define BME280_CTRL_MEAS_REG                                        0xF4  /*Ctrl Measure Register */
#define BME280_SOFT_RESET                                           0xB6  /*Soft Reset Value*/
#define BME280_NORMAL_MODE                                          0x03  /*Normal Mode Value*/
#define BME280_OVERSAMP_1X                                          0x20  /*Temperature Oversampling Value*/
#define BME280_TEMP_REG_ADDR                                        0xFA  /*Temperature Register*/
#define BME280_CHIP_ID_REG_ADDR                                     0xD0
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
	SENSOR_APP_STATE_INIT=0,
	SENSOR_APP_STATE_SENSOR_INIT,
    SENSOR_APP_STATE_WAIT_INIT_COMPLETE,
    SENSOR_APP_STATE_WAIT_MEAS_REQ,
    SENSOR_APP_STATE_WAIT_MEAS_COMPLETE,
    SENSOR_APP_STATE_ERROR,

} SENSOR_APP_STATES;

typedef enum
{
    SENSOR_APP_SPI_BUFFER_STATUS_COMPLETE = 0,
    SENSOR_APP_SPI_BUFFER_STATUS_ERROR,
    SENSOR_APP_SPI_BUFFER_STATUS_INVALID,

}SENSOR_APP_SPI_BUFFER_STATUS;

typedef struct
{
    uint8_t                             wrBuffer[10];
    uint8_t                             nBytes;
    SENSOR_APP_SPI_BUFFER_STATUS	    bufferStatus;
    DRV_SPI_BUFFER_HANDLE               bufferHandle;
} SENSOR_APP_SPI_WR_REQ;

typedef struct
{
    uint8_t                             wrBuffer[10];
    uint8_t                             nWrBytes;
    uint8_t                             rdBuffer[10];
    uint8_t                             nRdBytes;
    SENSOR_APP_SPI_BUFFER_STATUS	    bufferStatus;
    DRV_SPI_BUFFER_HANDLE               bufferHandle;
} SENSOR_APP_SPI_WR_RD_REQ;

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
    SENSOR_APP_STATES           state;
    DRV_HANDLE                  handle;
    SENSOR_APP_SPI_WR_REQ       sensorControlData;
    SENSOR_APP_SPI_WR_RD_REQ    calibrationData;
    SENSOR_APP_SPI_WR_RD_REQ    temperatureData;
    uint8_t                     sensorCalibValues[6];
    volatile bool               readTemperatureReq;
    float                       temperature;


	/* Un-comment below line of code for Lab 3 ---------> (Lab 3_1) */
    SemaphoreHandle_t 	xSemaphore;

    /* TODO: Define any additional data used by the application. */

} SENSOR_APP_DATA;


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
    void SENSOR_APP_Initialize ( void )

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
    SENSOR_APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void SENSOR_APP_Initialize ( void );


/*******************************************************************************
  Function:
    void SENSOR_APP_Tasks ( void )

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
    SENSOR_APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void SENSOR_APP_Tasks( void );
float SENSOR_APP_GetTemperature(void);

#endif /* _SENSOR_APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

