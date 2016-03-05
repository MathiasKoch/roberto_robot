#ifndef _I2CBus_h
#define _I2CBus_h


#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "ros/ros.h"

#define MAX_WRITE_LEN                   255
#define MAX_READ_LEN                    255

class I2CBus{
    public:
        I2CBus();
        ~I2CBus();

        unsigned char m_I2CBus;                                 // I2C bus of the imu (eg 1 for Raspberry Pi usually)

        bool I2COpen();
        void I2CClose();
        bool I2CRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                     unsigned char *data, const char *errorMsg);    // normal read with register select
        bool I2CRead(unsigned char slaveAddr, unsigned char length,
                     unsigned char *data, const char *errorMsg);    // read without register select
        bool I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                      unsigned char length, unsigned char const *data, const char *errorMsg);
        bool I2CWrite(unsigned char slaveAddr, unsigned char regAddr,
                      unsigned char const data, const char *errorMsg);

        void delayMs(int milliSeconds);

    protected:
        bool I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg);

    private:
        int m_I2C;
        unsigned char m_currentSlave;
};

#endif