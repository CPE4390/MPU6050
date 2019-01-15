#include <xc.h>
#include "MPU6050.h"


int pic18_i2c_enable(void) {
    return 0;
}

int pic_18_i2c_disable(void) {
    return 0;
}

int pic_18_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    
    return 0;
}

int pic18_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    
    return 0;
}

int get_ms(unsigned long *count) {
    *count = 0;
    return 0;
}