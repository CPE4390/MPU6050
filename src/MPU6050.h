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

    int pic18_i2c_enable(void);
    int pic18_i2c_disable(void);
    int pic18_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
    int pic18_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);
    
#define _XTAL_FREQ      32000000ul
#define delay_ms        __delay_ms
    int get_ms(unsigned long *count);
    
#define MPU6050

#ifdef	__cplusplus
}
#endif

#endif	/* MPU6050_H */

