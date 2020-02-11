/*
 * test_vl53l0x.c
 *
 *  Created on: 5 Feb 2020
 *      Author: ugochi
 */
#include "test_vl53l0x.h"

bool_t begin(uint8_t i2c_addr, bool_t debug) {

	int32_t status_int;
	int32_t init_done = 0;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	    uint8_t VhvSettings;
	    uint8_t PhaseCal;

	    pMyDevice->I2cDevAddr      = 0x52;
	        pMyDevice->comms_type      =  1;
	        pMyDevice->comms_speed_khz =  400;

	        pMyDevice->i2c = i2c;

	         pMyDevice->i2c->begin();     // VL53L0X_i2c_init();

//	         if( pVersion->major != VERSION_REQUIRED_MAJOR ||
//	                     pVersion->minor != VERSION_REQUIRED_MINOR ||
//	                     pVersion->build != VERSION_REQUIRED_BUILD )
//	                 {
//	                     printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
//	                         pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
//	                         VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
//	                 }

	         Status = VL53L0X_DataInit(&MyDevice); // Data initialization

	         Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
	                 if(Status == VL53L0X_ERROR_NONE)
	                 {
	                     printf("VL53L0X_GetDeviceInfo:\n");
	                     printf("Device Name : %s\n", DeviceInfo.Name);
	                     printf("Device Type : %s\n", DeviceInfo.Type);
	                     printf("Device ID : %s\n", DeviceInfo.ProductId);
	                     printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
	                 printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

	                 if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
	                 	printf("Error expected cut 1.1 but found cut %d.%d\n",
	                                DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
	                         Status = VL53L0X_ERROR_NOT_SUPPORTED;
	                     }
	                 }

	                 if(Status == VL53L0X_ERROR_NONE)
	                    {
	                        printf ("Call of VL53L0X_StaticInit\n");
	                        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
	                       // print_pal_error(Status);
	                    }

	                    if(Status == VL53L0X_ERROR_NONE)
	                    {
	                        printf ("Call of VL53L0X_PerformRefCalibration\n");
	                        Status = VL53L0X_PerformRefCalibration(pMyDevice,
	                        		&VhvSettings, &PhaseCal); // Device Initialization
	                       // print_pal_error(Status);
	                    }

	                    if(Status == VL53L0X_ERROR_NONE)
	                    {
	                        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
	                        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
	                        		&refSpadCount, &isApertureSpads); // Device Initialization
	                        printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
	                        //print_pal_error(Status);
	                    }

	                    if(Status == VL53L0X_ERROR_NONE)
	                    {

	                        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
	                        printf ("Call of VL53L0X_SetDeviceMode\n");
	                        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
	                       // print_pal_error(Status);
	                    }

	                    // Enable/Disable Sigma and Signal check
	                    if (Status == VL53L0X_ERROR_NONE) {
	                        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
	                        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	                    }
	                    if (Status == VL53L0X_ERROR_NONE) {
	                        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
	                        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	                    }

	                    if (Status == VL53L0X_ERROR_NONE) {
	                        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
	                        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
	                    }

	                    if (Status == VL53L0X_ERROR_NONE) {
	                        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
	                        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
	                        		(FixPoint1616_t)(1.5*0.023*65536));
	                    }


}

VL53L0X_Error getSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t *RangingMeasurementData, bool_t debug)
	        {
	Status = VL53L0X_ERROR_NONE;
	FixPoint1616_t  LimitCheckCurrent;

	if( Status == VL53L0X_ERROR_NONE) {

            printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);
	}


      print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
            		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

            printf("RANGE IGNORE THRESHOLD: %f\n\n", (float)LimitCheckCurrent/65536.0);

            printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);

    return Status;


}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n", RangeStatus, buf);

}



//  printf("Messi is goat!\n");
//	  VL53L0X_RangingMeasurementData_t measure;
//	  printf("measurement...");
//	  if(measure.RangeStatus != 4 ) {
//		  printf("distance (mm): ",measure.RangeMilliMeter);

//	  strcpy((char*)data, "MESSI!\r\n");
//	  HAL_UART_Transmit(&huart2, data, sizeof(data), 100);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
//	 HAL_Delay(5000);

	//  printf("test for sensor with nucleo\n");
	//  if(!sensor.begin()) {
	//	  printf("boot of sensor failed");
//	  }

//   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

//  HAL_I2C_Master_Transmit(&hi2c1, 0xA0, data, 1, 100);
//  HAL_I2C_Master_Transmit(&hi2c1, 0xA0, data, 1, 100);
//  check = HAL_I2C_Master_Receive(&hi2c1, 0xA1, data, 2, 100);
//  checking = (data[0] << 8);
//  checking2 = (data[1]);
// uint16_t temp_data = (checking | checking2);
//  if (temp_data > 0x36B0)
//  {
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
//  }
