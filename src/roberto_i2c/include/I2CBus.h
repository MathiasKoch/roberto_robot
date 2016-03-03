#ifndef _I2CBus_h
#define _I2CBus_h

#include <stdint.h>
#include <unistd.h>
#include <string.h>

class I2CBus{
    public:
        I2CBus(const char * deviceName);
        ~I2CBus();

        void addressSet(uint8_t address);
        void writeByte(uint8_t command, uint8_t data);
        void write(const char * buf, uint32_t len);
        uint8_t readByte(uint8_t command);
        int tryReadByte(uint8_t command);
        void readBlock(uint8_t command, uint8_t size, uint8_t * data);

        uint8_t tryRead(uint8_t address);

        int tryReadByte(uint8_t address, uint8_t command){
            addressSet(address);
            return tryReadByte(command);
        }

    private:
        int fd;
};

#endif