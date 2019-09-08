/*
* Licensee agrees that the example code provided to Licensee has been developed and released by Bosch solely as an example to be used as a potential reference for application development by Licensee. 
* Fitness and suitability of the example code for any use within application developed by Licensee need to be verified by Licensee on its own authority by taking appropriate state of the art actions and measures (e.g. by means of quality assurance measures).
* Licensee shall be responsible for conducting the development of its applications as well as integration of parts of the example code into such applications, taking into account the state of the art of technology and any statutory regulations and provisions applicable for such applications. Compliance with the functional system requirements and testing there of (including validation of information/data security aspects and functional safety) and release shall be solely incumbent upon Licensee. 
* For the avoidance of doubt, Licensee shall be responsible and fully liable for the applications and any distribution of such applications into the market.
* 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are 
* met:
* 
*     (1) Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer. 
* 
*     (2) Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.  
*     
*     (3)The name of the author may not be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*----------------------------------------------------------------------------*/

/**
 * @ingroup APPS_LIST
 *
 * @defgroup SEND_VIRTUALSENSOR_DATA_OVER_USB SendVirtualSensorDataOverUsb
 * @{
 *
 * @brief Demo application of printing sensor data on serialport
 *
 * @details Demo application of printing all the defined sensors on serialport (USB virtual comport)
 * every configured interval (#PRINT_DATA_INTERVAL)
 * Sensors can be selected via a compile-time switch in the application. #sensorToPrint
 * \section VirtualSensors
 * Following are the Virtual Sensors that are supported currently.
 * \subsection Rotation
 * Test Procedure:
 *  -# Enable "Rotation" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Observe Rotation Sensor data value on the TeraTerm.
 * \subsection Linear Acceleration
 * Test Procedure:
 *  -# Enable "LinearAccel" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Start moving the XDK in all axis.
 *  -# Observe linear acceleration Sensor data value on the TeraTerm.
 * \subsection Gravity
 * Test Procedure:
 *  -# Enable "Gravity" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Start moving the XDK in all axis.
 *  -# Observe gravity Sensor data value on the TeraTerm
 * \subsection Gesture
 * Test Procedure:
 *  -# Enable "Gesture" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Start tilting the XDK in all the directions.
 *  -# Observe the number of tilts detected on the TeraTerm
 * \subsection CompassData
 * Test Procedure:
 *  -# Enable "Compass" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Observe Compass Sensor data value on the TeraTerm.
 * \subsection Humidity
 * Test Procedure:
 *  -# Enable "AbsoluteHumidity" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Observe Humidity Sensor data value on the TeraTerm.
 * \subsection StepCounter
 * Test Procedure:
 *  -# Enable "StepCounter" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Observe Step counter data value on the TeraTerm.
 * \subsection CalibratedAccelerometer
 * Test Procedure:
 *  -# Enable "CalibratedAccel" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Keep the XDK board flat on the table for few seconds. Yellow LED will start blinking continuously.
 *  -# Start rotating XDK to +/-90 degree in all axis and Wait for 5 sec in each axis.
 *  -# Observe Calibrated Accelerometer Sensor data value on the TeraTerm.
 * \subsection CalibratedGyroscope
 * Test Procedure:
 *  -# Enable "CalibratedGyro" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Keep the XDK board flat on the table for few seconds. Yellow LED will start blinking continuously.
 *  -# Start rotating XDK to +/-90 degree in all axis.
 *  -# Observe Calibrated Gyroscope Sensor data value on the TeraTerm.
 * \subsection CalibratedMagnetometer
 * Test Procedure:
 * The below steps has to follow to test the Calibrated Magnetometer Sensor.
 *  -# Enable "CalibratedMag" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Keep the XDK board flat on the table for few seconds. Yellow LED will start blinking continuously.
 *  -# Start rotating XDK to +/-90 degrees in all axis.
 *  -# Observe Calibrated Magnetometer Sensor data value on the TeraTerm.
 * \subsection MagneticFingerPrint
 * Test Procedure:
 *  -# Enable "FingerPrint" variable in VirtualSensors structure(VirtualSensor_Enable_T)
 *  -# Compiled and flash the bin file to XDK board.
 *  -# Procedure to experiment this application can be found in the attached image please see below
 * \image html FingerPrintUI.png
 *
 * @file
 **/

/* module includes ********************************************************** */

/* own header files */
#include "XdkAppInfo.h"
#undef BCDS_MODULE_ID  /* Module ID define before including Basics package*/
#define BCDS_MODULE_ID XDK_APP_MODULE_ID_APP_CONTROLLER

/* own header files */
#include "AppController.h"

/* system header files */
#include <stdio.h>

/* additional interface header files */
#include "XDK_LED.h"
#include "XDK_Button.h"
#include "XDK_VirtualSensor.h"
#include "XDK_Utils.h"
#include "BCDS_CmdProcessor.h"
#include "FreeRTOS.h"
#include "task.h"

/* constant definitions ***************************************************** */

/* local variables ********************************************************** */
static void Button1Callback(ButtonEvent_T buttonEvent);
static void Button2Callback(ButtonEvent_T buttonEvent);

static Button_Setup_T ButtonSetup =
        {
                .CmdProcessorHandle = NULL,
                .InternalButton1isEnabled = true,
                .InternalButton2isEnabled = true,
                .InternalButton1Callback = Button1Callback,
                .InternalButton2Callback = Button2Callback,
        };/**< Button setup parameters */

/** @note More than one sensor is not supported */
const VirtualSensor_Enable_T VirtualSensors =
        {
                .Rotation = 0,
                .Compass = 0,
                .AbsoluteHumidity = 0,
                .CalibratedAccel = 1,
                .CalibratedGyro = 0,
                .CalibratedMag = 0,
                .Gravity = 0,
                .StepCounter = 0,
                .FingerPrint = 0,
                .LinearAccel = 0,
                .Gesture = 0
        };

#define MATCHING_FACTOR  0.9f   /**< Macro represents fingerprint matching factor */

/** enum to represent user interface menu level */
typedef enum menuLevel_E
{
    UI_MENU_LEVEL_0,
    UI_MENU_LEVEL_1,
    UI_MENU_LEVEL_2,
    UI_MENU_LEVEL_3,
} MenuLevel_T;

/** enum to represent magnetic fingerprint operational status */
typedef enum FingPrintMonitoringState_E
{
    PROCESS_OFF,
    PROCESS_ON,
} FingPrintMonitoringState_T;

typedef enum FingPrintSetValue_E
{
    NO_REQUEST,
    VALUE_RESET_REQUEST,
    VALUE_SET_REQUEST,
} FingPrintSetValue_T;

static uint8_t ButtonLeftPressedCnt = UINT8_C(0);/**< counter to monitor menu level: 0-> mag fing print monitoring; 1->set/reset/check MF1; 2->set/reset/check MF2; 3->set/reset/check MF3 */

static FingPrintMonitoringState_T FingPrintStatus = PROCESS_OFF;/**< variable to store magnetic fingerprint oparation status */

static FingPrintSetValue_T FingPrintRequest[FINGERPRINT_REFVAL_MAX] = { NO_REQUEST, NO_REQUEST, NO_REQUEST };/**< variable to store whether a fingerprint has been requested  */

static CmdProcessor_T * AppCmdProcessor;/**< Handle to store the main Command processor handle to be used by run-time event driven threads */

static xTaskHandle AppControllerHandle = NULL;/**< OS thread handle for Application controller to be used by run-time blocking threads */

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

/**
 * @brief Process the Button2 Data in side the Application context
 *
 * @param[in]    buttonEvent
 *
 */
static void Button2Callback(ButtonEvent_T buttonEvent)
{
    Retcode_T retcode = RETCODE_OK;
    switch (buttonEvent)
    {
    case BUTTON_EVENT_PRESSED:
        if ((uint8_t)UI_MENU_LEVEL_0 == ButtonLeftPressedCnt)
        {
            /* pause/resume magnetic finger print monitoring process */

            if (PROCESS_ON == FingPrintStatus)
            {
                FingPrintStatus = PROCESS_OFF;
                printf("pause MFP monitoring\r\n");
            }
            else if (PROCESS_OFF == FingPrintStatus)
            {
                FingPrintStatus = PROCESS_ON;
                printf("resume MFP monitoring\r\n");
            }
        }
        else if ((uint8_t)UI_MENU_LEVEL_1 == ButtonLeftPressedCnt)
        {/* Set/Reset magnetic finger print 1 */

            if (VALUE_SET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_1])
            {
                FingPrintRequest[FINGERPRINT_REFVAL_1] = VALUE_RESET_REQUEST;
                printf("Level1 reset request\r\n");
            }
            else
            {
                FingPrintRequest[FINGERPRINT_REFVAL_1] = VALUE_SET_REQUEST;
                printf("Level1 set request\r\n");
            }
        }
        else if ((uint8_t)UI_MENU_LEVEL_2 == ButtonLeftPressedCnt)
        {/* Set/Reset magnetic finger print 2 */

            if (VALUE_SET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_2])
            {
                FingPrintRequest[FINGERPRINT_REFVAL_2] = VALUE_RESET_REQUEST;
                printf("Level2 reset request\r\n");
            }
            else
            {
                FingPrintRequest[FINGERPRINT_REFVAL_2] = VALUE_SET_REQUEST;
                printf("Level2 set request\r\n");
            }
        }
        else if ((uint8_t)UI_MENU_LEVEL_3 == ButtonLeftPressedCnt)
        {/* Set/Reset magnetic finger print 3 */

            if (VALUE_SET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_3])
            {
                FingPrintRequest[FINGERPRINT_REFVAL_3] = VALUE_RESET_REQUEST;
                printf("Level3 reset request\r\n");
            }
            else
            {
                FingPrintRequest[FINGERPRINT_REFVAL_3] = VALUE_SET_REQUEST;
                printf("Level3 set request\r\n");
            }
        }
        else
        {
            printf("Menu not available \n\r");
        }
        break;

    case BUTTON_EVENT_RELEASED:
        printf("PB2 Released\n\r");
        break;

    default:
        printf("Button1Callback : Unsolicited button event occurred for PB1 \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
        break;
    }
    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
    }
}

/**
 * @brief Process the Button1 Data in side the Application context
 *
 * @param[in]    buttonEvent
 *
 */
static void Button1Callback(ButtonEvent_T buttonEvent)
{
    Retcode_T retcode = RETCODE_OK;
    switch (buttonEvent)
    {
    case BUTTON_EVENT_PRESSED:
        if (ButtonLeftPressedCnt < (uint8_t) UI_MENU_LEVEL_3)
        {
            ButtonLeftPressedCnt++;
            printf("MFP button left pressed. Menu level: %d\r\n", ButtonLeftPressedCnt);
        }
        else
        {
            ButtonLeftPressedCnt = (uint8_t) UI_MENU_LEVEL_0;
            printf("MFP button left pressed. Menu level: %d\r\n", ButtonLeftPressedCnt);
        }
        break;

    case BUTTON_EVENT_RELEASED:
        printf("PB1 Released\n\r");
        break;

    default:
        printf("Button1Callback : Unsolicited button event occurred for PB2 \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_INVALID_PARAM);
        break;
    }
    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
    }
}

static void ProcessFingerPrint(void)
{
    Retcode_T fingerPrintretcode = RETCODE_OK;
    Retcode_T retcode = RETCODE_OK;
    VirtualSensor_FingerPrintStorageState_T fingPrintPrintingStatus = FINGERPRINT_STORAGE_EMPTY;
    VirtualSensor_DataType_T data[FINGERPRINT_REFVAL_MAX];
    bool fingPrintLedStatus[FINGERPRINT_REFVAL_MAX] = { false, false, false };

    /* check UI menu level */
    switch (ButtonLeftPressedCnt)
    {
    /* Magnetic finger print monitoring process */
    case UI_MENU_LEVEL_0:
        retcode = LED_Off(LED_INBUILT_RED);
        if (RETCODE_OK == retcode)
        {
            retcode = LED_Off(LED_INBUILT_ORANGE);
        }
        if (RETCODE_OK == retcode)
        {
            retcode = LED_Off(LED_INBUILT_YELLOW);
        }
        if (retcode != RETCODE_OK)
        {
            printf("Setting LED state failed\r\n");
        }
        if (PROCESS_ON == FingPrintStatus)
        {
            /* Magnetic fingeprint evaluation */
            fingerPrintretcode = VirtualSensor_MonitorFingerPrint(&data[FINGERPRINT_REFVAL_1]);

            /* Visualize when sensor measurement and fingerprint matching estimation */
            for (uint32_t fpIndex = (uint32_t) FINGERPRINT_REFVAL_1; fpIndex < (uint32_t) FINGERPRINT_REFVAL_MAX; fpIndex++)
            {
                if (data[fpIndex].FingerPrintMonitorData.DistanceMatching < MATCHING_FACTOR)
                {
                    fingPrintLedStatus[fpIndex] = false;
                }
                else
                {
                    fingPrintLedStatus[fpIndex] = true;
                }
            }
            retcode = (true == fingPrintLedStatus[FINGERPRINT_REFVAL_1]) ? LED_On(LED_INBUILT_RED) : LED_Off(LED_INBUILT_RED);
            if (RETCODE_OK == retcode)
            {
                retcode = (true == fingPrintLedStatus[FINGERPRINT_REFVAL_2]) ? LED_On(LED_INBUILT_ORANGE) : LED_Off(LED_INBUILT_ORANGE);
            }
            if (RETCODE_OK == retcode)
            {
                retcode = (true == fingPrintLedStatus[FINGERPRINT_REFVAL_3]) ? LED_On(LED_INBUILT_YELLOW) : LED_Off(LED_INBUILT_YELLOW);
            }
            if (retcode != RETCODE_OK)
            {
                printf("Setting LED state failed\r\n");
            }
        }
        break;

        /* Check status magnetic finger print 1*/
    case UI_MENU_LEVEL_1:

        fingerPrintretcode = VirtualSensor_CheckFingerPrintStoredValue(FINGERPRINT_REFVAL_1, &fingPrintPrintingStatus);

        if (FINGERPRINT_STORAGE_RECORDED == fingPrintPrintingStatus)
        {
            retcode = LED_On(LED_INBUILT_RED);
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_ORANGE);
            }
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_YELLOW);
            }
            if (retcode != RETCODE_OK)
            {
                printf("Setting LED state failed\r\n");
            }
        }
        else
        {
            retcode = LED_Toggle(LED_INBUILT_RED);
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_ORANGE);
            }
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_YELLOW);
            }
            if (retcode != RETCODE_OK)
            {
                printf("Setting LED state failed\r\n");
            }
        }

        if (VALUE_SET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_1])
        {
            /* Set magnetic fingerprint 1 */
            if (FINGERPRINT_STORAGE_EMPTY == fingPrintPrintingStatus)
            {
                fingerPrintretcode = (fingerPrintretcode + VirtualSensor_SetFingerPrintValue(FINGERPRINT_REFVAL_1));
            }
            else if (FINGERPRINT_STORAGE_RECORDED == fingPrintPrintingStatus)
            {
                printf("Fingerprint 1 recorded\r\n");
            }
        }
        if (VALUE_RESET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_1])
        {
            /* Reset magnetic fingerprint 1 */
            fingerPrintretcode = (fingerPrintretcode + VirtualSensor_ResetFingerPrintValue(FINGERPRINT_REFVAL_1));
            FingPrintRequest[FINGERPRINT_REFVAL_1] = NO_REQUEST;
            printf("Fingerprint 1 reset 1\r\n");
        }
        break;

        /* Check status magnetic finger print 2 */
    case UI_MENU_LEVEL_2:

        fingerPrintretcode = VirtualSensor_CheckFingerPrintStoredValue(FINGERPRINT_REFVAL_2, &fingPrintPrintingStatus);

        if (FINGERPRINT_STORAGE_RECORDED == fingPrintPrintingStatus)
        {
            retcode = LED_Off(LED_INBUILT_RED);
            if (RETCODE_OK == retcode)
            {
                retcode = LED_On(LED_INBUILT_ORANGE);
            }
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_YELLOW);
            }
            if (retcode != RETCODE_OK)
            {
                printf("Setting LED state failed\r\n");
            }
        }
        else
        {
            retcode = LED_Off(LED_INBUILT_RED);
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Toggle(LED_INBUILT_ORANGE);
            }
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_YELLOW);
            }
            if (retcode != RETCODE_OK)
            {
                printf("Setting LED state failed\r\n");
            }
        }

        if (VALUE_SET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_2])
        {
            /* Set magnetic fingerprint 2 */
            if (FINGERPRINT_STORAGE_EMPTY == fingPrintPrintingStatus)
            {
                fingerPrintretcode = (fingerPrintretcode + VirtualSensor_SetFingerPrintValue(FINGERPRINT_REFVAL_2));
            }
            else if (FINGERPRINT_STORAGE_RECORDED == fingPrintPrintingStatus)
            {
                printf("Fingerprint 2 set\r\n");
            }
        }
        if (VALUE_RESET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_2])
        {
            /* Reset magnetic fingerprint 2 */
            fingerPrintretcode = (fingerPrintretcode + VirtualSensor_ResetFingerPrintValue(FINGERPRINT_REFVAL_2));
            FingPrintRequest[FINGERPRINT_REFVAL_2] = NO_REQUEST;
            printf("Fingerprint 2 reset\r\n");
        }
        break;

        /* Check status magnetic finger print 3 */
    case UI_MENU_LEVEL_3:

        fingerPrintretcode = VirtualSensor_CheckFingerPrintStoredValue(FINGERPRINT_REFVAL_3, &fingPrintPrintingStatus);

        if (FINGERPRINT_STORAGE_RECORDED == fingPrintPrintingStatus)
        {
            retcode = LED_Off(LED_INBUILT_RED);
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_ORANGE);
            }
            if (RETCODE_OK == retcode)
            {
                retcode = LED_On(LED_INBUILT_YELLOW);
            }
            if (retcode != RETCODE_OK)
            {
                printf("Setting LED state failed\r\n");
            }
        }
        else
        {
            retcode = LED_Off(LED_INBUILT_RED);
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Off(LED_INBUILT_ORANGE);
            }
            if (RETCODE_OK == retcode)
            {
                retcode = LED_Toggle(LED_INBUILT_YELLOW);
            }
            if (retcode != RETCODE_OK)
            {
                printf("Setting LED state failed\r\n");
            }
        }

        if (VALUE_SET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_3])
        {
            /* Set magnetic fingerprint 3 */
            if (FINGERPRINT_STORAGE_EMPTY == fingPrintPrintingStatus)
            {
                fingerPrintretcode = (fingerPrintretcode + VirtualSensor_SetFingerPrintValue(FINGERPRINT_REFVAL_3));
            }
            else if (FINGERPRINT_STORAGE_RECORDED == fingPrintPrintingStatus)
            {
                printf("Fingerprint 3 set\r\n");
            }
        }
        if (VALUE_RESET_REQUEST == FingPrintRequest[FINGERPRINT_REFVAL_3])
        {
            /* Reset magnetic fingerprint 3 */
            fingerPrintretcode = (fingerPrintretcode + VirtualSensor_ResetFingerPrintValue(FINGERPRINT_REFVAL_3));
            FingPrintRequest[FINGERPRINT_REFVAL_3] = NO_REQUEST;
            printf("Fingerprint 3 reset\r\n");
        }
        break;
    default:
        printf("Menu not available \n\r");
        break;
    }
    if (RETCODE_OK != fingerPrintretcode)
    {
        Retcode_RaiseError(retcode);
    }

}

static void PrintRotation(void)
{
    Retcode_T retcode;
    VirtualSensor_DataType_T rotationData;

    retcode = VirtualSensor_GetRotationData(&rotationData, ROTATION_QUATERNION);
    if (RETCODE_OK == retcode)
    {
        printf("Rotation - Quaternion : %3.2f %3.2f %3.2f %3.2f\n\r",
                rotationData.RotationQuaternion.W, rotationData.RotationQuaternion.X, rotationData.RotationQuaternion.Y, rotationData.RotationQuaternion.Z);
    }
    else
    {
        printf("Rotation Data Value Read failed \n\r");
    }
}

static void PrintCompass(void)
{
    Retcode_T retcode;
    float compassData = 0.0f;

    retcode = VirtualSensor_GetCompassData(&compassData);
    if (RETCODE_OK == retcode)
    {
        printf("Compass Data : %3.2f\n\r", compassData);
    }
    else
    {
        printf("Compass Data Value Read failed \n\r");
    }
}

static void PrintHumidity(void)
{
    Retcode_T retcode;
    float humidityData = 0.0f;

    retcode = VirtualSensor_GetAbsoluteHumidityData(&humidityData);
    if (RETCODE_OK == retcode)
    {
        printf("Absolute Humidity= %3.1f g/m3\n\r", humidityData);
    }
    else
    {
        printf("Actual Humidity Read Data Failed\n\r");
    }
}

static void PrintCalibratedMag(void)
{
    Retcode_T retcode;
    VirtualSensor_DataType_T data;
    retcode = VirtualSensor_GetCalibratedMag(&data, MAG_LSB_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED MAG DATA - LSB\t: %ld lsb\t%ld lsb\t%ld lsb\n\r",
                (long int) data.AxisInteger.X, (long int) data.AxisInteger.Y, (long int) data.AxisInteger.Z);
    }
    else
    {
        printf("Calibrated Magnetometer lsb Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetCalibratedMag(&data, MAG_GAUSS_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED MAG DATA - uT\t: %.4f uT\t%.4f uT\t%.4f uT\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Calibrated Magnetometer micro tesla Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetCalibratedMag(&data, MAG_MICROTESLA_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED MAG DATA - GAUSS\t: %.4f gauss\t%.4f gauss\t%.4f gauss\n\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Calibrated Magnetometer Gauss Value Read failed \n\r");
    }
}

static void PrintCalibratedGyro(void)
{
    Retcode_T retcode;
    VirtualSensor_DataType_T data;
    retcode = VirtualSensor_GetCalibratedGyro(&data, GYRO_LSB_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED GYRO DATA - LSB\t: %ld lsb\t%ld lsb\t%ld lsb\n\r",
                (long int) data.AxisInteger.X, (long int) data.AxisInteger.Y, (long int) data.AxisInteger.Z);
    }
    else
    {
        printf("Calibrated Gyroscope lsb Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetCalibratedGyro(&data, GYRO_RPS_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED GYRO DATA - RAD/SEC\t: %.4f rad/s\t%.4f rad/s\t%.4f rad/s\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Calibrated Gyroscope radians per second Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetCalibratedGyro(&data, GYRO_DPS_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED GYRO DATA - DEG/SEC\t: %.4f deg/s\t%.4f deg/s\t%.4f deg/s\n\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Calibrated Gyroscope degrees per second Value Read failed \n\r");
    }
}

static void PrintCalibratedAccel(void)
{
    Retcode_T retcode;
    VirtualSensor_DataType_T data;
    retcode = VirtualSensor_GetCalibratedAccel(&data, ACCEL_LSB_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED ACCEL DATA - LSB\t: %ld lsb\t%ld lsb\t%ld lsb\n\r",
                (long int) data.AxisInteger.X, (long int) data.AxisInteger.Y, (long int) data.AxisInteger.Z);
    }
    else
    {
        printf("Calibrated Accelerometer lsb Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetCalibratedAccel(&data, ACCEL_MPS2_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED ACCEL DATA - M/S2\t: %.4f m/s2\t%.4f m/s2\t%.4f m/s2\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Calibrated Accelerometer metre per second squared Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetCalibratedAccel(&data, ACCEL_G_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("CALIBRATED ACCEL DATA - G\t: %.4f g\t%.4f g\t%.4f g\n\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Calibrated Accelerometer G Value Read failed \n\r");
    }
}

static void PrintGravity(void)
{
    Retcode_T retcode;
    VirtualSensor_DataType_T data;
    retcode = VirtualSensor_GetGravity(&data, GRAVITY_LSB_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("GRAVITY DATA - LSB\n\rx=%ld\n\ry =%ld\n\rz =%ld\n\r",
                (long int) data.AxisInteger.X, (long int) data.AxisInteger.Y, (long int) data.AxisInteger.Z);
    }
    else
    {
        printf("Gravity lsb Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetGravity(&data, GRAVITY_MPS2_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("GRAVITY DATA - M/S2\n\rx=%.4f m/s2\n\ry=%.4f m/s2\n\rz=%.4f m/s2 \n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Gravity metre per second squared Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetGravity(&data, GRAVITY_G_MODE);
    if (RETCODE_OK == retcode)
    {
        printf("GRAVITY DATA - G\n\rx=%.4f g\n\ry=%.4f g \n\rz=%.4f g\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Gravity G Value Read failed \n\r");
    }
}

static void PrintStepCounter(void)
{
    Retcode_T retcode;
    int16_t stepCounterValue = 0;

    retcode = VirtualSensor_GetStepCounter(&stepCounterValue);
    if (RETCODE_OK == retcode)
    {
        /* Print step count information */
        printf("Step Count = %d \n\r", stepCounterValue);
    }
    else
    {
        printf("Step counter Data Value Read failed \n\r");
    }
}

static void PrintLinearAccel(void)
{
    Retcode_T retcode;
    VirtualSensor_DataType_T data;
    retcode = VirtualSensor_GetLinearAccel(&data, ACCEL_LSB_MODE);
    if (RETCODE_OK == retcode)
    {
        /* Print linear acceleration lsb information */
        printf("Linear Acceleration Data in LSB\n\rx=%ld\n\ry =%ld\n\rz =%ld\n\r",
                (long int) data.AxisInteger.X, (long int) data.AxisInteger.Y, (long int) data.AxisInteger.Z);
    }
    else
    {
        printf("Linear Acceleration lsb Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetLinearAccel(&data, ACCEL_MPS2_MODE);
    if (RETCODE_OK == retcode)
    {
        /* Print linear acceleration lsb information */
        printf("Linear Acceleration Data in M/S2\n\rx=%.4f m/s2\n\ry=%.4f m/s2\n\rz=%.4f m/s2 \n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Linear Acceleration metre per second squared Value Read failed \n\r");
    }

    retcode = VirtualSensor_GetLinearAccel(&data, ACCEL_G_MODE);
    if (RETCODE_OK == retcode)
    {
        /* Print linear acceleration lsb information */
        printf("Linear Acceleration Data in G\n\rx=%.4f g\n\ry=%.4f g \n\rz=%.4f g\n\r",
                (float) data.AxisFloat.X, (float) data.AxisFloat.Y, (float) data.AxisFloat.Z);
    }
    else
    {
        printf("Linear Acceleration G Value Read failed \n\r");
    }
}

static void PrintGesture(void)
{
    Retcode_T retcode = RETCODE_OK;
    uint32_t gestureCounterValue = UINT32_C(0);
    static uint32_t gestureCount = UINT32_C(0);

    retcode = VirtualSensor_GetGestureCount(&gestureCounterValue);
    if ((RETCODE_OK == retcode) && (gestureCounterValue != 0))
    {
        /* Print gesture count information */
        if (gestureCounterValue == 1)
        {
            gestureCount++;
            printf("Tilt detected = %ld \n\r", (long int) gestureCount);
        }
    }
}

/**
 * @brief Print the enabled VirtualSensor for every PRINT_DATA_INTERVAL mS
 * - LED
 * - Button
 *
 * @param[in] pvParameters
 * Unused
 *
 */
static void AppControllerFire(void* pvParameters)
{
    BCDS_UNUSED(pvParameters);

    while (1)
    {
        if (VirtualSensors.Rotation)
        {
            PrintRotation();
        }
        else if (VirtualSensors.Compass)
        {
            PrintCompass();
        }
        else if (VirtualSensors.AbsoluteHumidity)
        {
            PrintHumidity();
        }
        else if (VirtualSensors.CalibratedAccel)
        {
            PrintCalibratedAccel();
        }
        else if (VirtualSensors.CalibratedGyro)
        {
            PrintCalibratedGyro();
        }
        else if (VirtualSensors.CalibratedMag)
        {
            PrintCalibratedMag();
        }
        else if (VirtualSensors.Gravity)
        {
            PrintGravity();
        }
        else if (VirtualSensors.StepCounter)
        {
            PrintStepCounter();
        }
        else if (VirtualSensors.FingerPrint)
        {
            ProcessFingerPrint();
        }
        else if (VirtualSensors.LinearAccel)
        {
            PrintLinearAccel();
        }
        else if (VirtualSensors.Gesture)
        {
            PrintGesture();
        }
        else
        {
            printf("Enable Virtual Sensor to be tested");
        }

        if (VirtualSensors.FingerPrint)
        {
            vTaskDelay(pdMS_TO_TICKS(FINGERPRINT_PROCESS_INTERVAL));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(PRINT_DATA_INTERVAL));
        }
    }
}

/**
 * @brief To enable the necessary modules for the application
 * - LED
 * - Button
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerEnable(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);
    Retcode_T retcode = RETCODE_OK;

    retcode = VirtualSensor_Enable();
    if (RETCODE_OK == retcode)
    {
        retcode = LED_Enable();
    }
    if ((RETCODE_OK == retcode) && (VirtualSensors.FingerPrint))
    {
        retcode = Button_Enable();
    }
    if (RETCODE_OK == retcode)
    {
        if (pdPASS != xTaskCreate(AppControllerFire, (const char * const ) "AppController", TASK_STACK_SIZE_APP_CONTROLLER, NULL, TASK_PRIO_APP_CONTROLLER, &AppControllerHandle))
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_OUT_OF_RESOURCES);
        }
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerEnable : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
    Utils_PrintResetCause();
}

/**
 * @brief To setup the necessary modules for the application
 * - LED
 * - Button
 *
 * @param[in] param1
 * Unused
 *
 * @param[in] param2
 * Unused
 */
static void AppControllerSetup(void * param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    Retcode_T retcode = RETCODE_OK;
    VirtualSensor_Setup_T setup;
    setup.Enable = VirtualSensors;
    retcode = VirtualSensor_Setup(&setup);
    if (RETCODE_OK == retcode)
    {
        retcode = LED_Setup();
    }
    if ((RETCODE_OK == retcode) && (VirtualSensors.FingerPrint))
    {
        ButtonSetup.CmdProcessorHandle = AppCmdProcessor;
        retcode = Button_Setup(&ButtonSetup);
    }
    if (RETCODE_OK == retcode)
    {

        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerEnable, NULL, UINT32_C(0));
    }
    if (RETCODE_OK != retcode)
    {
        printf("AppControllerSetup : Failed \r\n");
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/* global functions ********************************************************* */

/** Refer interface header for description */
void AppController_Init(void * cmdProcessorHandle, uint32_t param2)
{
    BCDS_UNUSED(param2);

    Retcode_T retcode = RETCODE_OK;

    if (cmdProcessorHandle == NULL)
    {
        printf("AppController_Init : Command processor handle is NULL \r\n");
        retcode = RETCODE(RETCODE_SEVERITY_ERROR, RETCODE_NULL_POINTER);
    }
    else
    {
        AppCmdProcessor = (CmdProcessor_T *) cmdProcessorHandle;
        retcode = CmdProcessor_Enqueue(AppCmdProcessor, AppControllerSetup, NULL, UINT32_C(0));
    }

    if (RETCODE_OK != retcode)
    {
        Retcode_RaiseError(retcode);
        assert(0); /* To provide LED indication for the user */
    }
}

/**@} */
/** ************************************************************************* */

