#include "I2CBus.h"
#include "ros/ros.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>


// TODO: I2C Mutex lock?

I2CBus::I2CBus(const char * deviceName){
    fd = open(deviceName, O_RDWR);
    if (fd == -1){
        ROS_ERROR("Failed to open I2C device, returned errno %s",strerror(errno));
    }
}

I2CBus::~I2CBus(){
    close(fd);
}

void I2CBus::addressSet(uint8_t address){
    int result = ioctl(fd, I2C_SLAVE, address);
    if (result == -1){
        ROS_ERROR("Failed to set I2C address, returned errno %s",strerror(errno));
    }
}

void I2CBus::writeByte(uint8_t command, uint8_t data){
    int result = i2c_smbus_write_byte_data(fd, command, data);
    if (result == -1){
        ROS_ERROR("Failed to write byte to I2C, returned errno %s",strerror(errno));
    }
}

void I2CBus::write(const char * buf, uint32_t len){
    int result = -1 ;
    
    if(!fd)
        return;

    // Do simple use of I2C smbus command regarding number of bytes to transfer
    // Write 1 byte
    if (len == 2)
        result = i2c_smbus_write_byte_data(fd, buf[0], buf[1]);
    // Write 1 word
    else if (len == 3)
        result = i2c_smbus_write_word_data(fd, buf[0], (buf[2]<<8) | buf[1] );
    // Write bulk data
    else 
        result = i2c_smbus_write_i2c_block_data(fd, buf[0], len-1, (const __u8 *) &buf[1]);

    if (result == -1){
        ROS_ERROR("Failed to write to I2C, returned errno %s",strerror(errno));
    }

}

uint8_t I2CBus::readByte(uint8_t command){
    int result = i2c_smbus_read_byte_data(fd, command);
    if (result == -1){
        ROS_ERROR("Failed to read byte from I2C, returned errno %s",strerror(errno));
    }
    return result;
}

int I2CBus::tryReadByte(uint8_t command){
    return i2c_smbus_read_byte_data(fd, command);
}

uint8_t I2CBus::tryRead(uint8_t address){
    addressSet(address);
    return i2c_smbus_read_byte(fd);
}

void I2CBus::readBlock(uint8_t command, uint8_t size, uint8_t * data){
    int result = i2c_smbus_read_i2c_block_data(fd, command, size, data);
    if (result != size){
        ROS_ERROR("Failed to read block from I2C, returned errno %s",strerror(errno));
    }
}