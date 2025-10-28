#ifndef __dynamic_calibration_h__
#define __dynamic_calibration_h__

void initializeCalibrationStorage();
void storeCalibrationPoint(float currentAccel, float actualRPS, bool switch3);
void printCalibrationData();
void clearCalibrationData();
float calculateActualRPS(float currentAccel);

#endif
