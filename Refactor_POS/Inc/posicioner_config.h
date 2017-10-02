#ifndef POSICIONER_CONFIG_H
#define POSICIONER_CONFIG_H

#define TIMER_X_AXIS_ENGINE TIM5
#define TIMER_Y1_AXIS_ENGINE TIM2
#define TIMER_Y2_AXIS_ENGINE TIM2

#define TIMER_X_AXIS_ENCODER TIM1
#define TIMER_Y1_AXIS_ENCODER TIM3
#define TIMER_Y2_AXIS_ENCODER TIM4

#define TIMER_X_AXIS_IRQn TIM5_IRQn
#define TIMER_Y1_AXIS_IRQn TIM2_IRQn
#define TIMER_Y2_AXIS_IRQn TIM2_IRQn

#define AXIS_X_FORWARD_DIR (HAL_GPIO_WritePin(X_Dir_GPIO_Port, X_Dir_Pin, GPIO_PIN_SET))
#define AXIS_X_BACKWARD_DIR (HAL_GPIO_WritePin(X_Dir_GPIO_Port, X_Dir_Pin, GPIO_PIN_RESET))

#define AXIS_Y1_FORWARD_DIR (HAL_GPIO_WritePin(Y_Dir_GPIO_Port, Y_Dir_Pin, GPIO_PIN_SET))
#define AXIS_Y1_BACKWARD_DIR (HAL_GPIO_WritePin(Y_Dir_GPIO_Port, Y_Dir_Pin, GPIO_PIN_RESET))






#define SYMBOL_NUM_AXIS 1 //number of symbols in axis
#define SYMBOL_NUM_CMD 2 //number of symbols in command

#define AXIS_NUM  2 // number of axis
#define COMMAND_NUM 25 //number of engines settings (speed, acceleration, deceleration, end point and oth.)

#define TIMER_FREQUENCY 108000000
#define MAX_PERIOD 70000 // Max period of timers
#define MIN_PERIOD 5000 // min perion of timers
#define PULSE_WIDTH 700 // width of PWMs puls (pulse/min_period <= 3/5)
#define PULSE_OFF 0 // Disable impulse of pwm


#define NUM_ENGINE_STEPS 400// steps of engines in one round
#define NUM_ENCODER_STEPS 2500// steps of encoder in one round
#define DRIVER_PRESCALLER 12// prescaller of driving

#define STEPS_IN_MM 100;
#define ENCODER_DIAMETR_MM 42.44
#define MICROSTEPS_IN_MM 			(STEPS_IN_MM * DRIVER_PRESCALLER) //steps in one mm
#define STEPS_IN_ROUND (NUM_ENGINE_STEPS * DRIVER_PRESCALLER) // engine steps number in one round of encoder
#define MM_IN_ONE_ENCODER_ROUND (ENCODER_DIAMETR_MM * 3.1416) //steps in one mm
#define MIN_SPEED (TIMER_FREQUENCY/MAX_PERIOD) // Minimum speed of engine (imulse/second)

#define MID_NUM 10


#define MIN_SETS_ACCELERATION 5
#define MAX_SETS_ACCELERATION 250

#define MIN_SETS_DECELERATION 5
#define MAX_SETS_DECELERATION 250

#define MIN_SETS_SPEED (1650/STEPS_IN_MM)
#define MAX_SETS_SPEED ((TIMER_FREQUENCY/MIN_PERIOD)/STEPS_IN_MM);

#define MIN_SETS_POSITION 20
#define MAX_SETS_POSITION 1500

#define ADMIT_ERROR_POSITION 3

#define MAX_ERROR_OF_ENCODER_MM 20



#define NETWORK_ADDRESS_FLASH_SECTOR FLASH_SECTOR_6
#define NETWORK_ADDRESS_FLASH_ADDR 0x08080000

#define NETWORK_PORT_FLASH_SECTOR FLASH_SECTOR_6
#define NETWORK_PORT_FLASH_ADDR  (NETWORK_ADDRESS_FLASH_ADDR + 4) 


#define UDPTR_ADDRESS_FLASH_SECTOR FLASH_SECTOR_7
#define UDPTR_ADDRESS_FLASH_ADDR 0x080c0000

#define UDPTR_PORT_FLASH_SECTOR FLASH_SECTOR_7
#define UDPTR_PORT_FLASH_ADDR  (UDPTR_ADDRESS_FLASH_ADDR + 4)

#define UDPTR_INTERVAL_FLASH_SECTOR FLASH_SECTOR_7
#define UDPTR_INTERVAL_FLASH_ADDR (UDPTR_PORT_FLASH_ADDR + 4)



#endif /* POSICIONER_CONFIG_H*/