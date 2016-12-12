/*
 RiBot firmware V1.0
 Firmware for the RiBot robot

*/

//--------------------
// Usage documentation
//--------------------

/** \file
	Implementation of the main file of the RiBot mobile robot firmware
*/



//---------
// Includes
//---------

#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#include <math.h>
#include <stdlib.h>

#include "RC5_IR_Emul_Receiver.h"

#include "stm32f10x_i2c.h"

#include "led_rgb.h"

#include "power_management.h"

/** Maximal absolute angle (-/+) for the beating caudal fin  */
#define MAX_ABSOLUTE_ANGLE 30.0

/** Angle per step of the stepper motor, given by datasheet of motor  */
#define MICROSTEP_ANGLE 0.171

/** Maximal frequency for the beating caudal fin  */
#define MAX_FREQUENCY 20.0          // Valid only for small movements

/** Maximal frequency of step for stepper motor  */
#define MAX_STEP_FREQUENCY 1200     // For 2400 max but we miss step->1200 is ok

/** Full periode for PWM 200 -> 100 % PWM  */
#define FULL_PERIOD 400             // For 100% PWM

/** definition of TRUE  */
#define TRUE 1

/** definition of FALSE  */
#define FALSE 0

/** definition of boolean value ??  */
#define bool int

#define LEDDROITE 0X38


#define LEDGAUCHE 0X39


#define USELED 0


/** Status of IR, if Yes->IR received  */
extern StatusYesOrNo RC5_FrameReceived;

/** The message received by IR  */
RC5Frame_TypeDef RC5_Frame;


/** previous IR received  */
uint8_t RC5_TogglePrevious = 0;

/** If first time an IR message has been received */
StatusYesOrNo FirstTimeIssued = YES;


// ui variables are the User interface variables that we must modify
//from the Live data emBlocks watches (only global variables can be seen)
// or via remote control

/** User interface variables, control of the tail: amplitude +/- 30°, */
/** frequency max 20 Hz, motor break instruction if fin is in contact with the PCB */
float ui_amplitude_angle = 20.0; 	// total angle is +/- amplitude_angle (max 30.0)
float ui_frequency = 1.0;			// if the frequency is to high for the max motor speed the amplitude will not be respected
bool ui_motor_breaked = TRUE;      // give the state of motors if they are breaked or not and allows to control them too

bool new_data_ready = FALSE;

int new_motor_speed = 0;
int new_motor_counter_init = 0;
int new_nbr_step_max = 0;

/** boolean for value of each phase  */
int MOTOR_PHA, MOTOR_PHB, MOTOR_PHC, MOTOR_PHD;

void actualizepinstate();

/** value to reduce the current consumption while motor is moving  */
int value1=450;
int value2=500;


//I2C_InitTypeDef* I2C1;

void InitializeGPIOOUT() {
	GPIO_InitTypeDef gpioStructure;


    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, DISABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // This was for Port B
	// 5 - PowerOn : Output, active low, OpenDrain, external 100K PU at VBat

    //gpioStructure.GPIO_Pin = GPIO_Pin_5;
    //gpioStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    //gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_Init(GPIOB, &gpioStructure);

    //GPIO_ResetBits(GPIOB, GPIO_Pin_5);

    gpioStructure.GPIO_Pin = GPIO_Pin_5;
    gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioStructure);

    GPIO_SetBits(GPIOA, GPIO_Pin_5);

	// Port A
	// 6 - /Break : Input, active low, internal PU
	// 8 - Phase A : Output, PushPull, external 100K PD in order to disable the H-bridge at power on
	// 9 - Phase /A
	// 10 - Phase B
	// 11 - Phase /B
	// 12 - IR_RC5

    gpioStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10 | GPIO_Pin_11;
    //gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioStructure);

    gpioStructure.GPIO_Pin = GPIO_Pin_6;
    gpioStructure.GPIO_Mode = GPIO_Mode_IPU;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioStructure);





    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);
}



void InitializeTimer() {
	// High speed internel clock at 8MHz

    TIM_TimeBaseInitTypeDef timerInitStructure;
    TIM_TimeBaseStructInit(&timerInitStructure);


    timerInitStructure.TIM_Prescaler = 33;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = FULL_PERIOD;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &timerInitStructure);


    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);


	TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_InActive;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = FULL_PERIOD;
    TIM_OCInitStruct.TIM_OutputNState=TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_OCNIdleState=TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);

    TIM_BDTRInitTypeDef breakInitStructure;
    TIM_BDTRStructInit(&breakInitStructure);

    breakInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    breakInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	breakInitStructure.TIM_Break = TIM_Break_Enable;
	breakInitStructure.TIM_DeadTime = 0;
	breakInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	breakInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRConfig(TIM1, &breakInitStructure);

	TIM_ITConfig(TIM1, TIM_IT_Update|TIM_IT_Break, ENABLE); //ENABLE/DISABLE the break

	TIM_Cmd(TIM1, ENABLE);

}

void EnableTimerInterrupt() {
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);


}

void TIM1_BRK_IRQHandler() {
    ui_motor_breaked = TRUE;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
}

/* Handler Timer for IR sensor*/
void TIM2_IRQHandler(){
    RC5_Sample_Data();
}

/* Handler External Interrupt for IR sensor*/
void EXTI15_10_IRQHandler(){
    RC5_MeasureFirstLowDuration();
}

void TIM1_UP_IRQHandler() {

    /** In the stack so activated only one time  */
    static int motor_speed = 0;
    static int motor_counter = 0;
    static int motor_counter_init;

	static int nbr_step_max = 0;
    static int position = 0;
    static int phase = 0;
    static int delta_step = 0;

    if (new_data_ready) {
        motor_speed = new_motor_speed;
        motor_counter_init = new_motor_counter_init;
        motor_counter = motor_counter_init;
        nbr_step_max = new_nbr_step_max;
    if(position > nbr_step_max)
        delta_step = -1;
    else if(position < nbr_step_max)
        delta_step = +1;
    else if (position > 0)
        delta_step = -1;
    else if(position < 0)
        delta_step = +1;
    new_data_ready = FALSE;
    }


     if (motor_speed != 0) {
        motor_counter--;
        if (motor_counter <= 0) {
            if(position >= abs(nbr_step_max)) {
                delta_step = -1;
            }
            else if(position <= -abs(nbr_step_max)) {
                delta_step = +1;
            }
            motor_counter = motor_counter_init;
            position += delta_step;
            phase += delta_step;
            if (delta_step > 0) {
                if (phase > 8)
                    phase = 1;
            }
            else if (phase < 1)
                phase = 8;
        }
    }
    else {
        phase = 0;
    }

    /** set the phase on the port pins */
    switch (phase)
    {
        case 0:
        MOTOR_PHA = 0;
        MOTOR_PHB = 0;
        MOTOR_PHC = 0;
        MOTOR_PHD = 0;
        break;
        case 1:
        MOTOR_PHA = 1;
        MOTOR_PHB = 0;
        MOTOR_PHC = 1;
        MOTOR_PHD = 0;
        break;
        case 2:
        MOTOR_PHA = 0;
        MOTOR_PHB = 0;
        MOTOR_PHC = 1;
        MOTOR_PHD = 0;
        break;
        case 3:
        MOTOR_PHA = 0;
        MOTOR_PHB = 1;
        MOTOR_PHC = 1;
        MOTOR_PHD = 0;
        break;
        case 4:
        MOTOR_PHA = 0;
        MOTOR_PHB = 1;
        MOTOR_PHC = 0;
        MOTOR_PHD = 0;
        break;
        case 5:
        MOTOR_PHA = 0;
        MOTOR_PHB = 1;
        MOTOR_PHC = 0;
        MOTOR_PHD = 1;
        break;
        case 6:
        MOTOR_PHA = 0;
        MOTOR_PHB = 0;
        MOTOR_PHC = 0;
        MOTOR_PHD = 1;
        break;
        case 7:
        MOTOR_PHA = 1;
        MOTOR_PHB = 0;
        MOTOR_PHC = 0;
        MOTOR_PHD = 1;
        break;
        case 8:
        MOTOR_PHA = 1;
        MOTOR_PHB = 0;
        MOTOR_PHC = 0;
        MOTOR_PHD = 0;
        break;
    }

    actualizepinstate();
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

int main()
{

    // internal variables in order to communicate with the interrupt handler
    static float amplitude_angle = 20.0; 	//total angle is +/- amplitude_angle
    static float frequency = 1.0;			//if the frequency is to high for the max motor speed it will not be respected

    static bool nbr_step_max_updated = FALSE;
    static bool motor_counter_init_updated = FALSE;
    static bool motor_breaked = FALSE;   // In order to enable the control of PWM Outputs

  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

    // Initialize GPIO

    InitializeGPIOOUT();
    //GPIO_SetBits(GPIOA, GPIO_Pin_5);

    /** initialize ADC */
    InitializeADC();

    /** initialize I2C */
    initializeI2C();

    setLedIRGB(LEDGAUCHE,0x28,0,0,0);
    setLedIRGB(LEDDROITE,0x28,0,0,0);

    setLedIRGB(LEDGAUCHE,0x2F,31,31,31);
    setLedIRGB(LEDDROITE,0x2F,31,31,31);

    /** Enable Timer Interrupt for IntI */
    EnableTimerInterrupt();

    /** Initialize Timer */
    InitializeTimer();

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

    /** Initialize IR Receiver */
    RC5_Receiver_Init();
    float measuredPower=0.0;

    while(1) {


        measuredPower=ReadADC();

        if(RC5_FrameReceived==YES)
        {
            RC5_Frame = RC5_Decode();

            /** If the first time RC5 frame is received */
            if(FirstTimeIssued == YES)
            {
              /** Get and invert the toggle bit received at the first time to allow
                to enter the next condition at the first time */
              RC5_TogglePrevious = ~(RC5_Frame.ToggleBit)&0x01;

              /** Initialize the variable to avoid to enter in this section next time */
              FirstTimeIssued = NO;
            }

            /** If the toggle bit value has changed then display the RC5 frame */
            if(RC5_Frame.ToggleBit != RC5_TogglePrevious)
            {

              RC5_TogglePrevious = RC5_Frame.ToggleBit;

              switch(RC5_Frame.Command)
              {
                case 1 :
                    ui_amplitude_angle=5.0;
                    ui_frequency=5.0;
                    if(USELED){
                        setLedIRGB(LEDGAUCHE,0x2F,31,0,0);
                    }
                    else{
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }

                    break;

                case 2 :
                    ui_amplitude_angle=ui_amplitude_angle+1.0;
                    if(USELED){
                        setLedIRGB(LEDGAUCHE,0x2F,0,0,16);
                    }
                    else{
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }
                    break;

                case 3:
                    ui_amplitude_angle=20.0;
                    ui_frequency=1.0;
                    if(USELED){
                        setLedIRGB(LEDGAUCHE,0x2F,0,10,0);
                    }
                    else{
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }
                    break;

                case 4 :
                    ui_frequency=ui_frequency-1.0;
                    break;

                case 5 :
                    ui_amplitude_angle=0.0;
                    ui_frequency=0.0;
                    if(USELED){
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }
                    else{
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }
                    if(ui_motor_breaked){
                        ui_motor_breaked=FALSE;
                    }
                    else
                    {
                        ui_motor_breaked=TRUE;
                    }
                    break;

                case 6 :
                    ui_frequency=ui_frequency+1.0;

                    break;

                case 7 :
                    ui_amplitude_angle=15.0;
                    ui_frequency=2.0;
                    if(USELED){
                        setLedIRGB(LEDDROITE,0x2F,31,0,0);
                    }
                    else{
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }
                    break;

                case 8 :
                    ui_amplitude_angle=ui_amplitude_angle-1.0;
                    if(USELED){
                        setLedIRGB(LEDDROITE,0x2F,0,10,0);
                    }
                    else{
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }
                    break;

                case 9:
                    ui_amplitude_angle=2.0;
                    ui_frequency=10;
                    if(USELED){
                        setLedIRGB(LEDDROITE,0x2F,0,0,16);
                    }
                    else{
                        setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                        setLedIRGB(LEDDROITE,0x28,0,0,0);
                    }

                    break;

                case 12 :
                    ui_amplitude_angle=0.0;
                    ui_frequency=0.0;
                    setLedIRGB(LEDDROITE,0x28,0,0,0);
                    setLedIRGB(LEDGAUCHE,0x28,0,0,0);
                    //GPIO_SetBits(GPIOB, GPIO_Pin_5);
                    GPIO_ResetBits(GPIOA, GPIO_Pin_5);

              }

            }
        }
        /*
        else
        {
            if(measuredPower<3.3)
            {
                setLedIRGB(LED2,0x28,10,10,0);
            }
            else if(measuredPower>3.8)
            {
                setLedIRGB(LED2,0x28,10,10,0);
            }
            else{
                setLedIRGB(LED2,0x28,10,0,10);
            }
        }*/

        /* computes the parameters to move the motor  */
        if (!new_data_ready) {
            // Actually don't manage single shot mode but only continuous movements
            // Compute new_nbr_step_max

            /** Saturate amplitude */
            if (ui_amplitude_angle > MAX_ABSOLUTE_ANGLE)
                    ui_amplitude_angle = MAX_ABSOLUTE_ANGLE;
            else if (ui_amplitude_angle < -MAX_ABSOLUTE_ANGLE)
                    ui_amplitude_angle = -MAX_ABSOLUTE_ANGLE;

            /** If we have a new amplitude, compute the number of steps necessary */
            /** for the fin to reach */
            if (ui_amplitude_angle != amplitude_angle) {
                amplitude_angle = ui_amplitude_angle;
                new_nbr_step_max = (int)(amplitude_angle / MICROSTEP_ANGLE);
                nbr_step_max_updated = TRUE;
            }
            else
                nbr_step_max_updated = FALSE;

            // Compute new_motor_speed and new_motor_counter_init

            /** Saturate frequency */
            if (ui_frequency > MAX_FREQUENCY)
                ui_frequency = MAX_FREQUENCY;

            /** Here there is a 4 because 4 tail motion left/right/right/left to have
            one beating of the tail  */
            if ((ui_frequency != frequency) | (nbr_step_max_updated)) {
                frequency = ui_frequency;
                new_motor_speed = (int)(frequency * new_nbr_step_max*4);
                if (new_motor_speed != 0) {
                    if (new_motor_speed > MAX_STEP_FREQUENCY){
                        new_motor_speed = MAX_STEP_FREQUENCY;
                        new_nbr_step_max = (int)(new_motor_speed / (frequency*4));
                    }
                    else if (new_motor_speed < -MAX_STEP_FREQUENCY){
                        new_motor_speed = -MAX_STEP_FREQUENCY;
                        new_nbr_step_max = (int)(new_motor_speed / (frequency*4));
                    }
                    new_motor_counter_init = abs(MAX_STEP_FREQUENCY / new_motor_speed);
                }
                else
                    new_motor_counter_init=10; //???
                motor_counter_init_updated = TRUE;
            }
            else
                motor_counter_init_updated = FALSE;

            new_data_ready = nbr_step_max_updated || motor_counter_init_updated;
        }
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
        /** Here calibration of the tail */
        /*
        if (!motor_breaked && ui_motor_breaked) {
            TIM_CtrlPWMOutputs(TIM1, DISABLE);
            motor_breaked = TRUE;
        }
        else if (motor_breaked && !ui_motor_breaked) {
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
            motor_breaked = FALSE;
        }*/
    }
}

/** Force Pin active/inactive */

void actualizepinstate()
{
	if(MOTOR_PHA == 1)
		TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_Active);
	else
		TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);

	if(MOTOR_PHB == 1)
		TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_Active);
	else
		TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);

	if(MOTOR_PHC == 1)
		TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_Active);
	else
		TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);

	if(MOTOR_PHD == 1)
		TIM_ForcedOC4Config(TIM1, TIM_ForcedAction_Active);
	else
		TIM_ForcedOC4Config(TIM1, TIM_ForcedAction_InActive);
}
