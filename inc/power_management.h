#ifndef _POWER_MANAGEMENT_H_
#define _POWER_MANAGEMENT_H_

/** Initialize the ARD */
void InitializeADC(void);

/** Initialize the ADC and return floating value of the voltage */
float ReadADC(void);

#endif
