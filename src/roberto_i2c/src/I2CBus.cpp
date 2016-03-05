#include "I2CBus.h"


// TODO: I2C Mutex lock?

I2CBus::I2CBus(){
    m_I2CBus = 255;
    m_currentSlave = 255;
    m_I2C = -1;
}

I2CBus::~I2CBus(){
    I2CClose();
}


bool I2CBus::I2COpen(){
    char buf[32];

    if (m_I2C >= 0)
        return true;

    if (m_I2CBus == 255) {
        ROS_ERROR("No I2C bus has been set\n");
        return false;
    }
    sprintf(buf, "/dev/i2c-%d", m_I2CBus);
    m_I2C = open(buf, O_RDWR);
    if (m_I2C < 0) {
        ROS_ERROR("Failed to open I2C bus %d\n", m_I2CBus);
        m_I2C = -1;
        return false;
    }
    
    return true;
}

void I2CBus::I2CClose(){
    if (m_I2C >= 0) {
        close(m_I2C);
        m_I2C = -1;
        m_currentSlave = 255;
    }
}

bool I2CBus::I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char const data, const char *errorMsg)
{
    return I2CWrite(slaveAddr, regAddr, 1, &data, errorMsg);
}

bool I2CBus::I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                   unsigned char length, unsigned char const *data, const char *errorMsg){
    int result;
    unsigned char txBuff[MAX_WRITE_LEN + 1];

    if (!I2CSelectSlave(slaveAddr, errorMsg))
        return false;


    if (length == 0) {
        result = write(m_I2C, &regAddr, 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                ROS_ERROR("I2C write of regAddr failed - %s\n", errorMsg);
            return false;
        } else if (result != 1) {
            if (strlen(errorMsg) > 0)
                ROS_ERROR("I2C write of regAddr failed (nothing written) - %s\n", errorMsg);
            return false;
        }
    } else {
        txBuff[0] = regAddr;
        memcpy(txBuff + 1, data, length);

        result = write(m_I2C, txBuff, length + 1);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                ROS_ERROR("I2C data write of %d bytes failed - %s\n", length, errorMsg);
            return false;
        } else if (result < (int)length) {
            if (strlen(errorMsg) > 0)
                ROS_ERROR("I2C data write of %d bytes failed, only %d written - %s\n", length, result, errorMsg);
            return false;
        }
    }
    return true;
}


bool I2CBus::I2CRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg){
    int tries, result, total;
    unsigned char rxBuff[MAX_READ_LEN + 1];

    if (!I2CWrite(slaveAddr, regAddr, 0, NULL, errorMsg))
        return false;

    total = 0;
    tries = 0;

    while ((total < length) && (tries < 5)) {
        result = read(m_I2C, data + total, length - total);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                ROS_ERROR("I2C read error from %d, %d - %s\n", slaveAddr, regAddr, errorMsg);
            return false;
        }

        total += result;

        if (total == length)
            break;

        //delayMs(10);
        tries++;
    }

    if (total < length) {
        if (strlen(errorMsg) > 0)
            ROS_ERROR("I2C read from %d, %d failed - %s\n", slaveAddr, regAddr, errorMsg);
        return false;
    }
    
    return true;
}

bool I2CBus::I2CRead(unsigned char slaveAddr, unsigned char length,
                    unsigned char *data, const char *errorMsg){
    int tries, result, total;
    unsigned char rxBuff[MAX_READ_LEN + 1];

    if (!I2CSelectSlave(slaveAddr, errorMsg))
        return false;

    total = 0;
    tries = 0;

    while ((total < length) && (tries < 5)) {
        result = read(m_I2C, data + total, length - total);

        if (result < 0) {
            if (strlen(errorMsg) > 0)
                ROS_ERROR("I2C read error from %d - %s\n", slaveAddr, errorMsg);
            return false;
        }

        total += result;

        if (total == length)
            break;

        //delayMs(10);
        tries++;
    }

    if (total < length) {
        if (strlen(errorMsg) > 0)
            ROS_ERROR("I2C read from %d failed - %s\n", slaveAddr, errorMsg);
        return false;
    }
   
    return true;
}


bool I2CBus::I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg)
{
    if (m_currentSlave == slaveAddr)
        return true;

    if (!I2COpen()) {
        ROS_ERROR("Failed to open I2C port - %s\n", errorMsg);
        return false;
    }

    if (ioctl(m_I2C, I2C_SLAVE, slaveAddr) < 0) {
        ROS_ERROR("I2C slave select %d failed - %s\n", slaveAddr, errorMsg);
        return false;
    }

    m_currentSlave = slaveAddr;

    return true;
}

void I2CBus::delayMs(int milliSeconds)
{
    usleep(1000 * milliSeconds);
}