/* 
 * File:   MPU6050.h
 * Author: bmcgarvey
 *
 * Created on January 15, 2019, 10:53 AM
 */

#ifndef MPU6050_H
#define	MPU6050_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
    //i2c functions
    int pic18_i2c_enable(void);
    int pic18_i2c_disable(void);
    int pic18_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
    int pic18_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
    
    //timing functions
    #define _XTAL_FREQ   32000000L
    extern unsigned long tickCount;
    int pic18_delay_ms(unsigned long num_ms);
    int pic18_get_ms(unsigned long *count);
    
    //DMP library defines
    #define MPU6050

#ifdef	__cplusplus
}
#endif

#endif	/* MPU6050_H */

