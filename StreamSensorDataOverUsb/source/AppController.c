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
 * @defgroup STREAM_SENSOR_DATA_OVER_USB StreamSensorDataOverUsb
 * @{
 *
 * @brief Application of printing all the defined sensors on serialport
 *
 * @details Demo application of printing all the defined sensors on serialport(USB virtual comport)
 *          every configured interval (#APP_CONTROLLER_TX_DELAY)
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
#include "XDK_Sensor.h"
#include "XDK_Utils.h"
#include "BCDS_Assert.h"
#include "BCDS_CmdProcessor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "XDK_Storage.h"
#include "XDK_LED.h"
#include "BCDS_SDCard_Driver.h"
#include "Serval_Clock.h"

/* constant definitions ***************************************************** */
#define BUFFER_SIZE                 UINT16_C(512)
#define SINGLE_BLOCK                UINT8_C(1)      /**< SD- Card Single block write or read */
#define DRIVE_ZERO                  UINT8_C(0)      /**< SD Card Drive 0 location */
#define SECTOR_VALUE                UINT8_C(6)      /**< SDC Disk sector value */
#define SINGLE_SECTOR_LEN           UINT32_C(512)   /**< Single sector size in SDcard */

/* local variables ********************************************************** */

Storage_Setup_T StorageSetupInfo =
        {
                .SDCard = true,
                .WiFiFileSystem = false
        };/**< Storage setup parameters */
static CmdProcessor_T *AppCmdProcessor;/**< Handle to store the main Command processor handle to be reused by ServalPAL thread */

static xTaskHandle AppControllerHandle = NULL;/**< OS thread handle for Application controller to be used by run-time blocking threads */

/**
 * @brief Callback function called when interrupt occurs
 *
 * @param [in]  param1
 * Unused
 *
 * @param [in]  param2
 * Unused
 */
static void AccelAppCallback(void *param1, uint32_t param2);
static void AppControllerEnable(void * param1, uint32_t param2);

/**
 * @brief Callback function called when interrupt occurs
 *
 * @param [in]  param1
 * Unused
 *
 * @param [in]  param2
 * Unused
 */
static void LightAppCallback(void *param1, uint32_t param2);

static Sensor_Setup_T SensorSetup =
        {
                .CmdProcessorHandle = NULL,
                .Enable =
                        {
                                .Accel = true,
                                .Mag = true,
                                .Gyro = true,
                                .Humidity = true,
                                .Temp = true,
                                .Pressure = true,
                                .Light = true,
                                .Noise = false,
                        },
                .Config =
                        {
                                .Accel =
                                        {
                                                .Type = SENSOR_ACCEL_BMA280,
                                                .IsRawData = false,
                                                .IsInteruptEnabled = true,
                                                .Callback = AccelAppCallback,
                                        },
                                .Gyro =
                                        {
                                                .Type = SENSOR_GYRO_BMG160,
                                                .IsRawData = false,
                                        },
                                .Mag =
                                        {
                                                .IsRawData = false,
                                        },
                                .Light =
                                        {
                                                .IsInteruptEnabled = true,
                                                .Callback = LightAppCallback,
                                        },
                                .Temp =
                                        {
                                                .OffsetCorrection = APP_TEMPERATURE_OFFSET_CORRECTION,
                                        },
                        },
        };/**< Sensor setup parameters */

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

static void AccelAppCallback(void *param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    printf("Accelerometer sensor application callback received\r\n");
}

static void LightAppCallback(void *param1, uint32_t param2)
{
    BCDS_UNUSED(param1);
    BCDS_UNUSED(param2);

    printf("*******Light sensor application callback received*******\r\n");
}
static Retcode_T AppControllerFatFileSystemWriteRead(const char* content, uint16_t fileSize)
{
    uint8_t ramBufferWrite[BUFFER_SIZE]; /* Temporary buffer for write file */
    uint8_t ramBufferRead[BUFFER_SIZE]; /* Temporary buffer for read file */
    const int8_t stringTestBuffer[fileSize];
    memcpy(stringTestBuffer, content, fileSize);

    static uint32_t writeOffset = 0, readOffset = 0;
    Retcode_T retcode = RETCODE_OK;
    for (uint32_t index = 0; index < fileSize; index++)
    {
        ramBufferWrite[index] = stringTestBuffer[index];
    }
    Storage_Read_T readCredentials =
            {
                    .FileName = TEST_FILENAME,
                    .ReadBuffer = ramBufferRead,
                    .BytesToRead = fileSize,
                    .ActualBytesRead = 0UL,
                    .Offset = 0UL,
            };
    Storage_Write_T writeCredentials =
            {
                    .FileName = TEST_FILENAME,
                    .WriteBuffer = ramBufferWrite,
                    .BytesToWrite = fileSize,
                    .ActualBytesWritten = 0UL,
                    .Offset = 0UL,
            };
    writeCredentials.Offset = writeOffset;
    retcode = Storage_Write(STORAGE_MEDIUM_SD_CARD, &writeCredentials);
    if (RETCODE_OK == retcode)
    {
    	printf("\n Write operation into SD card successful\n");
        if (writeCredentials.BytesToWrite == writeCredentials.ActualBytesWritten)
        {
            writeOffset = writeOffset + writeCredentials.BytesToWrite;
            readCredentials.Offset = readOffset;
            retcode = Storage_Read(STORAGE_MEDIUM_SD_CARD, &readCredentials);
        }
        else
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, FILE_WRITE_ERROR);
        }
    }
    if (RETCODE_OK == retcode)
    {
        if (readCredentials.BytesToRead == readCredentials.ActualBytesRead)

        {
            readOffset = writeOffset - readCredentials.BytesToRead;
        }
        else
        {
            retcode = RETCODE(RETCODE_SEVERITY_ERROR, FILE_READ_ERROR);
        }
    }
    if (RETCODE_OK == retcode)
    {
        for (uint8_t index = 0; index < fileSize; index++)
        {
            if ((ramBufferWrite[index]) != (ramBufferRead[index]))
            {
                /* Error compare buffers*/
                retcode = RETCODE(RETCODE_SEVERITY_ERROR, SDCARD_APP_ERROR);
            }
        }
    }
    return (retcode);
}

/**
 * @brief This function controls the application flow
 * - Triggers Sensor data sampling
 * - Read the sampled Sensor data
 *
 * @param[in] pvParameters
 * Unused
 */
static void AppControllerFire(void* pvParameters)
{
	BCDS_UNUSED(pvParameters);

	Retcode_T retcode = RETCODE_OK;
	Sensor_Value_T sensorValue;
	Retcode_T ledRetcode = RETCODE_OK;
	bool sdcardEject = false, status = false, sdcardInsert = false;

	retcode = Storage_IsAvailable(STORAGE_MEDIUM_SD_CARD, &status);
	if ((RETCODE_OK == retcode) && (true == status))
	{
		sdcardInsert = true;
		printf("SD card is inserted in XDK\n\r");
		ledRetcode = LED_On(LED_INBUILT_RED);
		if (RETCODE_OK != ledRetcode)
		{
			printf("SD card is inserted LED indication failure XDK\n\r");
		}
		if (sdcardEject == true)
		{
			retcode = Storage_Disable(STORAGE_MEDIUM_SD_CARD);
			if (RETCODE_OK == retcode)
			{
				retcode = Storage_Enable();
			}
			if (RETCODE_OK == retcode)
			{
				sdcardEject = false;
			}
		}
	}
	else
	{
		if (Retcode_GetCode(retcode) == (Retcode_T) RETCODE_STORAGE_SDCARD_UNINITIALIZED)
		{
			printf("\r\n SD card is not inserted in XDK\n\r");
			retcode = Storage_Enable();
		}
		else
		{
			if (true == sdcardInsert)
			{
				sdcardEject = true;
			}
			sdcardInsert = false;
			printf("\r\nSD card is removed from XDK\n\r");
			ledRetcode = LED_Off(LED_INBUILT_RED);
			if (RETCODE_OK != ledRetcode)
			{
				printf("SD card is not inserted LED indication failure XDK\n\r");
			}

		}
	}
	if (RETCODE_OK != retcode)
	{
		Retcode_RaiseError(retcode);
		printf("\n **************************Error Code = %d", retcode);
		//exit(0);
	}
	char humidity[10] = "\0";
	char pressure[10] = "\0";
	char temp[10] = "\0";
	char light[10] = "\0";
	char timestamp[20] = "\0";
	while (1)
	{

		memset(&sensorValue, 0x00, sizeof(sensorValue));

		retcode = Sensor_GetData(&sensorValue);
		if (RETCODE_OK == retcode)
		{
			if (SensorSetup.Enable.Accel && !(SensorSetup.Config.Accel.IsRawData))
			{
//				printf("Accelerometer Converted data :\n\rx =%ld mg\n\ry =%ld mg\n\rz =%ld mg\r\n",
//						(long int) sensorValue.Accel.X,
//						(long int) sensorValue.Accel.Y,
//						(long int) sensorValue.Accel.Z);
			}
			if (SensorSetup.Enable.Accel && (SensorSetup.Config.Accel.IsRawData))
			{
//				printf("Accelerometer Raw data :\n\rx =%ld \n\ry =%ld\n\rz =%ld\r\n",
//						(long int) sensorValue.Accel.X,
//						(long int) sensorValue.Accel.Y,
//						(long int) sensorValue.Accel.Z);
			}
			if (SensorSetup.Enable.Humidity)
			{
				printf("BME280 Environmental Conversion Data for Humidity:\n\rh =%ld %%rh\r\n",
						(long int) sensorValue.RH);
				sprintf(humidity,"%ld",(long int) sensorValue.RH);
			}
			if (SensorSetup.Enable.Pressure)
			{
				printf("BME280 Environmental Conversion Data for Pressure :\n\rp =%ld Pa\r\n",
						(long int) sensorValue.Pressure);
				sprintf(pressure,"%ld",(long int) sensorValue.Pressure);
			}
			if (SensorSetup.Enable.Temp)
			{
				printf("BME280 Environmental Conversion Data for temperature :\n\rt =%ld mDeg\r\n",
						(long int) sensorValue.Temp);
				sprintf(temp,"%ld",(long int) sensorValue.Temp);
			}
			if (SensorSetup.Enable.Mag && !(SensorSetup.Config.Mag.IsRawData))
			{
//				printf("Magnetometer Converted data :\n\rx =%ld microTesla\n\ry =%ld microTesla\n\rz =%ld microTesla\n\rr =%ld\r\n",
//						(long int) sensorValue.Mag.X,
//						(long int) sensorValue.Mag.Y,
//						(long int) sensorValue.Mag.Z,
//						(long int) sensorValue.Mag.R);
			}
			if (SensorSetup.Enable.Mag && SensorSetup.Config.Mag.IsRawData)
			{
//				printf("Magnetometer Raw data :\n\rx =%ld\n\ry =%ld\n\rz =%ld \n\rr =%ld\r\n",
//						(long int) sensorValue.Mag.X,
//						(long int) sensorValue.Mag.Y,
//						(long int) sensorValue.Mag.Z,
//						(long int) sensorValue.Mag.R);
			}
			if (SensorSetup.Enable.Gyro && !(SensorSetup.Config.Gyro.IsRawData))
			{
//				printf("Gyro Converted Data :\n\rx =%ld mDeg\n\ry =%ld mDeg\n\rz =%ld mDeg\r\n",
//						(long int) sensorValue.Gyro.X,
//						(long int) sensorValue.Gyro.Y,
//						(long int) sensorValue.Gyro.Z);
			}
			if (SensorSetup.Enable.Gyro && SensorSetup.Config.Gyro.IsRawData)
			{
//				printf("Gyro Raw Data :\n\rx =%ld \n\ry =%ld \n\rz =%ld \r\n",
//						(long int) sensorValue.Gyro.X,
//						(long int) sensorValue.Gyro.Y,
//						(long int) sensorValue.Gyro.Z);
			}
			if (SensorSetup.Enable.Light)
			{
				printf("Light sensor data obtained in millilux :%d \n\r", (unsigned int) sensorValue.Light);
				sprintf(light, "%d", (unsigned int) sensorValue.Light);
			}
			if (SensorSetup.Enable.Noise)
			{
				//printf("Noise Sensor RMS Voltage :\r\nVrms = %f \r\n", sensorValue.Noise);
			}
			uint64_t time;
			Clock_getTimeMillis(&time);
			//Clock_getTime(&time);
			printf("*****************time: %llu\n\r", time);
			sprintf(timestamp, "%llu", time);
			uint16_t sizeOfData = strlen(humidity) + strlen(pressure) + strlen(light) + strlen(temp) + strlen(timestamp) + 6;
			char *strData = (char*) malloc(sizeOfData * sizeof(char));
			sprintf(strData, "%s,%s,%s,%s,%s\r\n", timestamp, humidity, pressure, temp, light);



			printf("\nData stored on SD card time,humidity,pressure,temperature,light= %s\n", strData);
			AppControllerFatFileSystemWriteRead(strData, sizeOfData);
			free(strData);
			strData = NULL;
			Storage_Close();
		}
		if (RETCODE_OK != retcode)
		{
			Retcode_RaiseError(retcode);
		}
		vTaskDelay(pdMS_TO_TICKS(APP_CONTROLLER_TX_DELAY));
	}
}

/**
 * @brief To enable the necessary modules for the application
 * - Sensor
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

    Retcode_T retcode, fileDeleteRetcode = RETCODE_OK;
        retcode = Storage_Enable();
        if ((Retcode_T) RETCODE_STORAGE_SDCARD_NOT_AVAILABLE == Retcode_GetCode((retcode)))
        {
            /* This is only a warning error. So we will raise and proceed */
            Retcode_RaiseError(retcode);
            retcode = RETCODE_OK; /* SD card was not inserted */
        }
        if (RETCODE_OK == retcode)
        {
            fileDeleteRetcode = Storage_Delete(STORAGE_MEDIUM_SD_CARD, TEST_FILENAME);
            if (RETCODE_OK != fileDeleteRetcode)
            {
                printf("File does not exist. \n\r");
            }
        }
        if (RETCODE_OK == retcode)
        {
            retcode = LED_Enable();
        }
    Retcode_T retcode1 = Sensor_Enable();
    if ((RETCODE_OK == retcode) && (RETCODE_OK == retcode1))
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
 * - Sensor
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

    SensorSetup.CmdProcessorHandle = AppCmdProcessor;
    Retcode_T retcode = Sensor_Setup(&SensorSetup);
    Retcode_T retcode2 = RETCODE_OK;
       retcode2 = Storage_Setup(&StorageSetupInfo);
       if (RETCODE_OK == retcode2)
       {
           retcode = LED_Setup();
       }
       if ((RETCODE_OK == retcode) && (RETCODE_OK == retcode2))
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

/* global functions ********************************************************** */

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
