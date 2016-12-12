#ifndef _LED_RGB_H_
#define _LED_RGB_H_

/** Initialize the I2C */
void initializeI2C(void);

/** set the LED state */
void setLedI2C(uint8_t,uint8_t);

/** function to set the LED color directly */
void setLedIRGB(uint8_t,uint8_t, uint8_t,uint8_t,uint8_t);

#endif


