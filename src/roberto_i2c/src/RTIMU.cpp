////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTIMU.h"
#include "ros/ros.h"

#include "RTIMUSettings.h"
//#include "CalLib.h"

#include "RTIMUMPU9250.h"

RTIMU *RTIMU::createIMU(RTIMUSettings *settings)
{
    switch (settings->m_imuType) {
	    case RTIMU_TYPE_MPU9250:
	        return new RTIMUMPU9250(settings);
			
	    case RTIMU_TYPE_AUTODISCOVER:
	        if (settings->discoverIMU(settings->m_imuType, settings->m_I2CSlaveAddress)) {
	            return RTIMU::createIMU(settings);
	        }
	        return NULL;

	    default:
	        return 0;
    }
}


RTIMU::RTIMU(RTIMUSettings *settings)
{	
    m_settings = settings;

    m_calibrationMode = false;
    m_calibrationValid = false;
}

RTIMU::~RTIMU()
{
}

void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;

    if (m_settings->m_compassCalValid) {
        //  find biggest range

        for (int i = 0; i < 3; i++) {
            if ((m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) > maxDelta)
                maxDelta = m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i);
        }
        if (maxDelta < 0) {
            ROS_ERROR("Error in compass calibration data");
            return;
        }
        maxDelta /= 2.0f;                                       // this is the max +/- range

        for (int i = 0; i < 3; i++) {
            delta = (m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
            m_compassCalOffset[i] = (m_settings->m_compassCalMax.data(i) + m_settings->m_compassCalMin.data(i)) / 2.0f;
        }
    }

    if (m_settings->m_compassCalValid) {
        ROS_INFO("Using min/max compass calibration");
    } else {
        ROS_INFO("min/max compass calibration not in use");
    }

    /*if (m_settings->m_compassCalEllipsoidValid) {
        ROS_INFO("Using ellipsoid compass calibration");
    } else {
        ROS_INFO("Ellipsoid compass calibration not in use");
    }*/

    if (m_settings->m_accelCalValid) {
        ROS_INFO("Using accel calibration");
    } else {
        ROS_INFO("Accel calibration not in use");
    }
}
