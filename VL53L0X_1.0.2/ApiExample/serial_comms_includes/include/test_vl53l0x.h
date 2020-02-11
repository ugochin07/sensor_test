/*
 * test_vl53l0x.h
 *
 *  Created on: 5 Feb 2020
 *      Author: ugochi
 */
#include "vl53l0x_api.h"
#ifndef INC_TEST_VL53L0X_H_
#define INC_TEST_VL53L0X_H_

bool_t begin(uint8_t i2c_addr, bool_t debug);
VL53L0X_Error getSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t *RangingMeasurementData, bool_t debug);

VL53L0X_Error getSingleRangingMeasurement( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData, bool_t debug);
   void          printRangeStatus( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData );

VL53L0X_Error   Status      = VL53L0X_ERROR_NONE; ///< indicates whether or not the sensor has encountered an error

VL53L0X_Dev_t                       MyDevice;
 VL53L0X_Dev_t                       *pMyDevice  = &MyDevice;
 VL53L0X_Version_t                   Version;
 VL53L0X_Version_t                   *pVersion   = &Version;
 VL53L0X_DeviceInfo_t                DeviceInfo;
#endif /* INC_TEST_VL53L0X_H_ */
