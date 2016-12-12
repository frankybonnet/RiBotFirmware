

#include <stm32f10x_gpio.h>
#include "led_rgb.h"


/** Clock frequency I2C */
#define I2C_SPEED 100000


void initializeI2C(){

    ///Initialize I2C, SCL: PB6 and SDA: PB7 with 100kHt serial clock
    //TM_I2C_Init(MyI2C, TM_I2C_PinsPack_1, 100000);

    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    I2C_Cmd(I2C1,ENABLE);

    /** I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /** I2C1 SDA and SCL configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /** SCL is pin06 and SDA is pin 07 for I2C1*/


    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    /** I2C1 configuration */

    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED ;
    I2C_Init(I2C1, &I2C_InitStructure);

}

void setLedI2C(uint8_t ledAddress,uint8_t valuePWM)
{
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    /* initiate start sequence */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* check start bit flag */

    //there is a timeout to avoid the program to stop here
    int timeout=0;

    while((!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))&&(timeout<100))
    {
        timeout=timeout+1;
    }
    /** send write command to chip */
    I2C_Send7bitAddress(I2C1, ledAddress<<1, I2C_Direction_Transmitter);

    /** check master is now in Tx mode */

    timeout=0;
    while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))&&(timeout<100))
    {
        timeout=timeout+1;
    }

    /** mode register address*/
    I2C_SendData(I2C1, valuePWM); //mean PWM1 with very low amplitude, 6F being the max
    /*wait for byte send to complete*/
    //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    ///*wait for byte send to complete*/
    timeout=0;
    while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))&&(timeout<100))
    {
        timeout=timeout+1;
    }

    /*clear bits*/

    /** generate stop*/
    I2C_GenerateSTOP(I2C1, ENABLE);
    /**stop bit flag*/
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

}

void setLedIRGB(uint8_t ledAddress,uint8_t intensity, uint8_t red,uint8_t green,uint8_t blue){

    if(red>32)
        red=32;
    if(green>32)
        green=32;
    if(blue>32)
        blue=32;

    setLedI2C(ledAddress,intensity);
    setLedI2C(ledAddress,red + 0x40 );
    setLedI2C(ledAddress,green + 0x60 );
    setLedI2C(ledAddress,blue + 0x80);
}


