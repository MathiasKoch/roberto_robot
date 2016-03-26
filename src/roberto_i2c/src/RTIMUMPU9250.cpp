////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
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

//  The MPU-9250 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)

#include "RTIMUMPU9250.h"
#include "RTIMUSettings.h"

#define COMPASS_ALPHA 0.2f

RTIMUMPU9250::RTIMUMPU9250(RTIMUSettings *settings) : RTIMU(settings)
{
    IMUinitialized = false;
}

RTIMUMPU9250::~RTIMUMPU9250()
{
}

unsigned long RTIMUMPU9250::millis(){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts );
    return ( ts.tv_sec * 1000 + ts.tv_nsec / 1000000L );
}

bool RTIMUMPU9250::setSampleRate(int rate)
{
    if ((rate < MPU9250_SAMPLERATE_MIN) || (rate > MPU9250_SAMPLERATE_MAX)) {
        ROS_ERROR("Illegal sample rate %d\n", rate);
        return false;
    }

    //  Note: rates interact with the lpf settings

    if ((rate < MPU9250_SAMPLERATE_MAX) && (rate >= 8000))
        rate = 8000;

    if ((rate < 8000) && (rate >= 1000))
        rate = 1000;

    if (rate < 1000) {
        int sampleDiv = (1000 / rate) - 1;
        m_sampleRate = 1000 / (1 + sampleDiv);
    } else {
        m_sampleRate = rate;
    }
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool RTIMUMPU9250::setGyroLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9250_GYRO_LPF_8800:
    case MPU9250_GYRO_LPF_3600:
    case MPU9250_GYRO_LPF_250:
    case MPU9250_GYRO_LPF_184:
    case MPU9250_GYRO_LPF_92:
    case MPU9250_GYRO_LPF_41:
    case MPU9250_GYRO_LPF_20:
    case MPU9250_GYRO_LPF_10:
    case MPU9250_GYRO_LPF_5:
        m_gyroLpf = lpf;
        return true;

    default:
        ROS_ERROR("Illegal MPU9250 gyro lpf %d\n", lpf);
        return false;
    }
}

bool RTIMUMPU9250::setAccelLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9250_ACCEL_LPF_1130:
    case MPU9250_ACCEL_LPF_460:
    case MPU9250_ACCEL_LPF_184:
    case MPU9250_ACCEL_LPF_92:
    case MPU9250_ACCEL_LPF_41:
    case MPU9250_ACCEL_LPF_20:
    case MPU9250_ACCEL_LPF_10:
    case MPU9250_ACCEL_LPF_5:
        m_accelLpf = lpf;
        return true;

    default:
        ROS_ERROR("Illegal MPU9250 accel lpf %d\n", lpf);
        return false;
    }
}


bool RTIMUMPU9250::setCompassRate(int rate)
{
    if ((rate < MPU9250_COMPASSRATE_MIN) || (rate > MPU9250_COMPASSRATE_MAX)) {
        ROS_ERROR("Illegal compass rate %d\n", rate);
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUMPU9250::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9250_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case MPU9250_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case MPU9250_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case MPU9250_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        ROS_ERROR("Illegal MPU9250 gyro fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUMPU9250::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9250_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case MPU9250_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case MPU9250_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case MPU9250_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        ROS_ERROR("Illegal MPU9250 accel fsr %d\n", fsr);
        return false;
    }
}


bool RTIMUMPU9250::IMUInit()
{
    unsigned char result;

    m_firstTime = true;

    //  configure IMU

    m_slaveAddr = m_settings->m_I2CSlaveAddress;

    setSampleRate(m_settings->m_MPU9250GyroAccelSampleRate);
    setCompassRate(m_settings->m_MPU9250CompassSampleRate);
    setGyroLpf(m_settings->m_MPU9250GyroLpf);
    setAccelLpf(m_settings->m_MPU9250AccelLpf);
    setGyroFsr(m_settings->m_MPU9250GyroFsr);
    setAccelFsr(m_settings->m_MPU9250AccelFsr);

    setCalibrationData();


    //  enable the bus

    if (!m_settings->I2COpen())
        return false;

    //  reset the MPU9250

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_PWR_MGMT_1, 0x80, "Failed to initiate MPU9250 reset"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_PWR_MGMT_1, 0x00, "Failed to stop MPU9250 reset"))
        return false;

    if (!m_settings->I2CRead(m_slaveAddr, MPU9250_WHO_AM_I, 1, &result, "Failed to read MPU9250 id"))
        return false;

    if (result != MPU9250_ID) {
        ROS_ERROR("Incorrect %s id 0x%x, expected 0x%x", IMUName(), result, MPU9250_ID);
        return false;
    }

    //  now configure the various components

    if (!setGyroConfig())
        return false;

    if (!setAccelConfig())
        return false;

    if (!setSampleRate())
        return false;

    if(!compassSetup()) {
        return false;
    }

    if (!setCompassRate())
        return false;

    //  enable the sensors



    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_PWR_MGMT_1, 1, "Failed to set pwr_mgmt_1"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_PWR_MGMT_2, 0, "Failed to set pwr_mgmt_2"))
         return false;

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return false;

    m_gyroAlpha = 1.0f / m_sampleRate;
    m_gyroStartTime = millis();
    m_gyroLearning = true;

    ROS_INFO("%s init complete\n", IMUName());
    IMUinitialized = true;
    return true;
}


bool RTIMUMPU9250::resetFifo()
{
    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_INT_ENABLE, 0, "Writing int enable"))
        return false;
    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_FIFO_EN, 0, "Writing fifo enable"))
        return false;
    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_USER_CTRL, 0, "Writing user control"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_USER_CTRL, 0x04, "Resetting fifo"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_USER_CTRL, 0x60, "Enabling the fifo"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_INT_ENABLE, 1, "Writing int enable"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_FIFO_EN, 0x78, "Failed to set FIFO enables"))
        return false;

    return true;
}

bool RTIMUMPU9250::setGyroConfig()
{
    unsigned char gyroConfig = m_gyroFsr + ((m_gyroLpf >> 3) & 3);
    unsigned char gyroLpf = m_gyroLpf & 7;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_GYRO_CONFIG, gyroConfig, "Failed to write gyro config"))
         return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_GYRO_LPF, gyroLpf, "Failed to write gyro lpf"))
         return false;
    return true;
}

bool RTIMUMPU9250::setAccelConfig()
{
    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_ACCEL_CONFIG, m_accelFsr, "Failed to write accel config"))
         return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_ACCEL_LPF, m_accelLpf, "Failed to write accel lpf"))
         return false;
    return true;
}

bool RTIMUMPU9250::setSampleRate()
{
    if (m_sampleRate > 1000)
        return true;                                        // SMPRT not used above 1000Hz

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_SMPRT_DIV, (unsigned char) (1000 / m_sampleRate - 1),
            "Failed to set sample rate"))
        return false;

    return true;
}

bool RTIMUMPU9250::compassSetup() {
    unsigned char asa[3];
    unsigned char result;

    /*bypassOn();
    //return true;

    if (!m_settings->I2CRead(AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &result, "Failed to read AK8963 id, Bypass may have failed!")){
        bypassOff();
        return false;
    }

    if (result != AK8963_DEVICEID) {
        bypassOff();
        ROS_ERROR("Incorrect %s id 0x%x, expected 0x%x", "AK8963 (Compass)", result, AK8963_DEVICEID);
        return false;
    }

    // get fuse ROM data

    if (!m_settings->I2CWrite(AK8963_ADDRESS, AK8963_CNTL, 0, "Failed to set compass in power down mode 1")) {
        bypassOff();
        return false;
    }

    if (!m_settings->I2CWrite(AK8963_ADDRESS, AK8963_CNTL, 0x0f, "Failed to set compass in fuse ROM mode")) {
        bypassOff();
        return false;
    }

    if (!m_settings->I2CRead(AK8963_ADDRESS, AK8963_ASAX, 3, asa, "Failed to read compass fuse ROM")) {
        bypassOff();
        return false;
    }

    if (!m_settings->I2CWrite(AK8963_ADDRESS, AK8963_CNTL, 0, "Failed to set compass in power down mode 2")) {
        bypassOff();
        return false;
    }

    bypassOff();*/

    

    //  convert asa to usable scale factor

    m_compassAdjust[0] = 1;//((float)asa[0] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[1] = 1;//((float)asa[1] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[2] = 1;//((float)asa[2] - 128.0) / 256.0 + 1.0f;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_MST_CTRL, 0x40, "Failed to set I2C master mode"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS, "Failed to set slave 0 address"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV0_REG, AK8963_ST1, "Failed to set slave 0 reg"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV0_CTRL, 0x88, "Failed to set slave 0 ctrl"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV1_ADDR, AK8963_ADDRESS, "Failed to set slave 1 address"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV1_REG, AK8963_CNTL, "Failed to set slave 1 reg"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV1_CTRL, 0x81, "Failed to set slave 1 ctrl"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV1_DO, 0x1, "Failed to set slave 1 DO"))
        return false;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_MST_DELAY_CTRL, 0x3, "Failed to set mst delay"))
        return false;

    return true;
}

bool RTIMUMPU9250::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;
    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_I2C_SLV4_CTRL, rate, "Failed to set slave ctrl 4"))
         return false;
    return true;
}

bool RTIMUMPU9250::getCompassCalValid(){
    return m_settings->m_compassCalValid;
}

bool RTIMUMPU9250::getAccelCalValid(){
    return m_settings->m_accelCalValid;
}


bool RTIMUMPU9250::bypassOn()
{
    unsigned char userControl;

    if (!m_settings->I2CRead(m_slaveAddr, MPU9250_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl &= ~0x20;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_INT_PIN_CFG, 0x82, "Failed to write int_pin_cfg reg"))
        return false;

    m_settings->delayMs(50);
    return true;
}


bool RTIMUMPU9250::bypassOff()
{
    unsigned char userControl;

    if (!m_settings->I2CRead(m_slaveAddr, MPU9250_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!m_settings->I2CWrite(m_slaveAddr, MPU9250_INT_PIN_CFG, 0x80, "Failed to write int_pin_cfg reg"))
         return false;

    m_settings->delayMs(50);
    return true;
}


int RTIMUMPU9250::IMUGetPollInterval()
{
    if (m_sampleRate > 400)
        return 1;
    else
        return (400 / m_sampleRate);
}

bool RTIMUMPU9250::IMURead()
{
    if(!IMUinitialized)
        return false;

    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[8];

    if (!m_settings->I2CRead(m_slaveAddr, MPU9250_FIFO_COUNT_H, 2, fifoCount, "Failed to read fifo count"))
         return false;

    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

    if (count == 512) {
        ROS_INFO("MPU-9250 fifo has overflowed");
        resetFifo();
        m_timestamp += m_sampleInterval * (512 / MPU9250_FIFO_CHUNK_SIZE + 1); // try to fix timestamp
        return false;
    }

    if (count > MPU9250_FIFO_CHUNK_SIZE * 40) {
        // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
        while (count >= MPU9250_FIFO_CHUNK_SIZE * 10) {
            if (!m_settings->I2CRead(m_slaveAddr, MPU9250_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
                return false;            
            count -= MPU9250_FIFO_CHUNK_SIZE;
            m_timestamp += m_sampleInterval;
        }
    }

    if (count < MPU9250_FIFO_CHUNK_SIZE){
        return false;
    }

    if (!m_settings->I2CRead(m_slaveAddr, MPU9250_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
        return false;

    if (!m_settings->I2CRead(m_slaveAddr, MPU9250_EXT_SENS_DATA_00, 8, compassData, "Failed to read compass data"))
        return false;

    RTMath::convertToVector(fifoData, m_accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_compass, 0.3f, false);

    if (m_gyroLearning) {
        // update gyro bias

        m_gyroBias.setX((1.0 - m_gyroAlpha) * m_gyroBias.x() + m_gyroAlpha * m_gyro.x());
        m_gyroBias.setY((1.0 - m_gyroAlpha) * m_gyroBias.y() + m_gyroAlpha * m_gyro.y());
        m_gyroBias.setZ((1.0 - m_gyroAlpha) * m_gyroBias.z() + m_gyroAlpha * m_gyro.z());

        if ((millis() - m_gyroStartTime) > 5000)
            m_gyroLearning = false;                     // only do this for 5 seconds
    }

    //  sort out gyro axes

    m_gyro.setX(m_gyro.x() - m_gyroBias.x());
    m_gyro.setY(-(m_gyro.y() - m_gyroBias.y()));
    m_gyro.setZ(-(m_gyro.z() - m_gyroBias.z()));

    //  sort out accel data;

    m_accel.setX(-m_accel.x());

    //  sort out compass axes

    float temp;

    temp = m_compass.x();
    m_compass.setX(m_compass.y());
    m_compass.setY(-temp);

    //  use the fuse data adjustments

    m_compass.setX(m_compass.x() * m_compassAdjust[0]);
    m_compass.setY(m_compass.y() * m_compassAdjust[1]);
    m_compass.setZ(m_compass.z() * m_compassAdjust[2]);

    //  calibrate if required

    if (!m_calibrationMode && m_calibrationValid) {
        m_compass.setX((m_compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        m_compass.setY((m_compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        m_compass.setZ((m_compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);
    }

    //  update running average

    m_compassAverage.setX(m_compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setY(m_compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setZ(m_compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));
    m_compass = m_compassAverage;

    if (m_firstTime)
        m_timestamp = millis();
    else
        m_timestamp += m_sampleInterval;

    m_firstTime = false;

    return true;
}


