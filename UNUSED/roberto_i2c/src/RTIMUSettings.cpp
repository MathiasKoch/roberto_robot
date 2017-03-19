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
#include "RTIMUMPU9250.h"

#define RATE_TIMER_INTERVAL 2

RTIMUSettings::RTIMUSettings()
{
    //  preset general defaults

    m_imuType = RTIMU_TYPE_AUTODISCOVER;
    m_I2CSlaveAddress = 0;

    //  MPU9250 defaults

    m_MPU9250GyroAccelSampleRate = 40;
    m_MPU9250CompassSampleRate = 20;
    m_MPU9250GyroLpf = MPU9250_GYRO_LPF_41;
    m_MPU9250AccelLpf = MPU9250_ACCEL_LPF_41;
    m_MPU9250GyroFsr = MPU9250_GYROFSR_1000;
    m_MPU9250AccelFsr = MPU9250_ACCELFSR_8;
}

/*RTIMUSettings::~RTIMUSettings(){

}*/

bool RTIMUSettings::discoverIMU(int& imuType, unsigned char& slaveAddress)
{
    unsigned char result;
    unsigned char altResult;

    if (I2COpen()){

        if (I2CRead(MPU9250_ADDRESS0, MPU9250_WHO_AM_I, &result, "")) {
            if (result == MPU9250_ID) {
                imuType = RTIMU_TYPE_MPU9250;
                slaveAddress = MPU9250_ADDRESS0;
                return true;
            }
        }

        if (I2CRead(MPU9250_ADDRESS1, MPU9250_WHO_AM_I, &result, "")) {
            if (result == MPU9250_ID) {
                imuType = RTIMU_TYPE_MPU9250;
                slaveAddress = MPU9250_ADDRESS1;
                return true;
            }
        }
    }

    return false;
}


bool RTIMUSettings::loadSettings(ros::NodeHandle * settings_nh_){
    int temp_int;

    settings_nh_->param("imu_type", m_imuType, 1);
    //settings_nh_->getParam("fusion_type", m_fusionType);


    settings_nh_->param("imu_slave_address", temp_int, 105);
    m_I2CSlaveAddress = (unsigned char) temp_int;

    //settings_nh_->getParam("axis_rotation", m_axisRotation);
    
    //double declination_radians;
    //settings_nh_->param("magnetic_declination", declination_radians, 0.0);
    //m_compassAdjDeclination = angles::to_degrees(declination_radians);


    //MPU9250
    settings_nh_->getParam("mpu9250/gyro_accel_sample_rate", m_MPU9250GyroAccelSampleRate);
    settings_nh_->getParam("mpu9250/compass_sample_rate", m_MPU9250CompassSampleRate);
    settings_nh_->getParam("mpu9250/accel_full_scale_range", m_MPU9250AccelFsr);
    settings_nh_->getParam("mpu9250/accel_low_pass_filter", m_MPU9250AccelLpf);
    settings_nh_->getParam("mpu9250/gyro_full_scale_range", m_MPU9250GyroFsr);
    settings_nh_->getParam("mpu9250/gyro_low_pass_filter", m_MPU9250GyroLpf);

    std::vector<double> compass_max, compass_min;
    if (settings_nh_->getParam("calib/compass_min", compass_min)
            && settings_nh_->getParam("calib/compass_max", compass_max)
            && compass_min.size() == 3 && compass_max.size() == 3)
    {
        m_compassCalMin = RTVector3(compass_min[0], compass_min[1], compass_min[2]);
        m_compassCalMax = RTVector3(compass_max[0],compass_max[1], compass_max[2]);
        m_compassCalValid = true;
        
        ROS_INFO("Got Calibration for Compass");
    }else{
        ROS_INFO("No Calibration for Compass");
    }
    
    std::vector<double> accel_max, accel_min;
    if (settings_nh_->getParam("calib/accel_min", accel_min)
            && settings_nh_->getParam("calib/accel_max", accel_max)
            && accel_min.size() == 3 && accel_max.size() == 3)
    {
        m_accelCalMin = RTVector3(accel_min[0], accel_min[1], accel_min[2]);
        m_accelCalMax = RTVector3(accel_max[0],accel_max[1], accel_max[2]);
        m_accelCalValid = true;
        
        ROS_INFO("Got Calibration for Accelerometer");
    }else{
        ROS_INFO("No Calibration for Accelerometer"); 
    }


    return true;
}