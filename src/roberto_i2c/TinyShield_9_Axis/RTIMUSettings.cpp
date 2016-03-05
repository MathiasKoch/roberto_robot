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

#include "RTIMUSettings.h"
#include "RTIMUMPU9150.h"

#define RATE_TIMER_INTERVAL 2

RTIMUSettings::RTIMUSettings()
{
    //  preset general defaults

    m_imuType = RTIMU_TYPE_AUTODISCOVER;
    m_I2CSlaveAddress = 0;

    //  MPU9150 defaults

    m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;
}

bool RTIMUSettings::discoverIMU(int& imuType, unsigned char& slaveAddress)
{
    unsigned char result;
    unsigned char altResult;

    if (I2Cdev::readByte(MPU9150_ADDRESS0, MPU9150_WHO_AM_I, &result)) {
        if (result == MPU9150_ID) {
            imuType = RTIMU_TYPE_MPU9150;
            slaveAddress = MPU9150_ADDRESS0;
            return true;
        }
    }

    if (I2Cdev::readByte(MPU9150_ADDRESS1, MPU9150_WHO_AM_I, &result)) {
        if (result == MPU9150_ID) {
            imuType = RTIMU_TYPE_MPU9150;
            slaveAddress = MPU9150_ADDRESS1;
            return true;
        }
    }

    return false;
}
