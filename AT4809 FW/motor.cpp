/****************************************************************************
**	INCLUDE
****************************************************************************/

//type definition using the bit width and signedness
#include <stdint.h>
//define the ISR routune, ISR vector, and the sei() cli() function
#include <avr/interrupt.h>
//name all the register and bit
#include <avr/io.h>
//General purpose macros
#include "at_utils.h"
//AT4809 PORT macros definitions
#include "at4809_port.h"
//Program wide definitions
#include "global.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

/****************************************************************************
**	PROTOTYPE GLOBAL VARS
****************************************************************************/

/****************************************************************************
**	GLOBAL VARS
****************************************************************************/

/****************************************************************************
**	FUNCTION
****************************************************************************/

/***************************************************************************/
//!	@brief function
//!	set_platform_pwm | int16_t | int16_t
/***************************************************************************/
//! @param right | int16_t | pwm to apply to the right side wheel(s)
//! @param left | int16_t | pwm to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong VNH7040 index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API for the VNH7040 driver
//! Control PWM of the platform to move according to the layout
/***************************************************************************/

bool set_platform_pwm( int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------
	
	//int16_t data[2];
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Select PWM controls
	g_control_mode_target = CONTROL_PWM;
	//Set target for PWM channels 0 and 1. Apply Layout correction
	g_pwm_target[0] = right;
	g_pwm_target[1] = left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_pwm | int16_t | int16_t


/***************************************************************************/
//!	@brief function
//!	set_platform_spd | int16_t | int16_t
/***************************************************************************/
//! @param right | int16_t | spd to apply to the right side wheel(s)
//! @param left | int16_t | spd to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong encoder index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API
//! Control SPD of the platform to move according to the layout
/***************************************************************************/

bool set_platform_spd( int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Select PWM controls
	g_control_mode_target = Control_mode::CONTROL_SLOW_SPD;
	//The PWM references are set as target for the PWM slope controller
	g_pid_spd_target[0] = right;
	g_pid_spd_target[1] = left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_spd | int16_t | int16_t

/***************************************************************************/
//!	@brief function
//!	set_platform_pos | int32_t | int32_t
/***************************************************************************/
//! @param right | int16_t | spd to apply to the right side wheel(s)
//! @param left | int16_t | spd to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong encoder index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API
//! Control SPD of the platform to move according to the layout
/***************************************************************************/

bool set_platform_pos( int32_t right, int32_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Select PWM controls
	g_control_mode_target = Control_mode::CONTROL_POS;
	//The PWM references are set as target for the PWM slope controller
	g_pid_pos_target[0] = right;
	g_pid_pos_target[1] = left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_pos | int32_t | int32_t

/****************************************************************************
//!	@brief function
**  set_vnh7040_pwm
****************************************************************************/
//! @param index	| uint8_t	| index of the motor to be controlled. 0 to 3
//! @param pwm		| int16_t	| Speed of the motor
//! @return bool	| false = OK | true = a wrong VNH7040 index was given
//! @brief Set direction and pwm setting of the VNH7040 controlled motor
//! @details
//!	HAL Hardware Abstraction Layer API for the VNH7040 driver
//! Set direction and pwm setting of the VNH7040 controlled motor
/***************************************************************************/

bool set_vnh7040_pwm( uint8_t index, int16_t pwm )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp direction
	uint8_t f_dir;
	//Temp 8bit pwm
	uint8_t vnh7040_pwm;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
		///STEP1: Compute direction
	//if: reverse
	if (pwm < 0)
	{
		//reverse
		f_dir = uint8_t(0xff);
		//correct sign
		pwm = -pwm;
	}
	//if: forward
	else
	{
		//forward
		f_dir = uint8_t(0x00);
	}
	
		///STEP2: Apply VNH7040 Hardware PWM limits
	//If: PWM setting is too low
	if (pwm <= MIN_VNH7040_PWM)
	{
		//Set PWM to zero. It would not result to motion anyway
		pwm = 0;
	}
	//If: PWM exceed maximum
	else if (pwm >= MAX_VNH7040_PWM)
	{
		//Clip PWM to maximum
		pwm = MAX_VNH7040_PWM;
	}
	//If: PWM is correct
	else
	{
		//do nothing
	}
	//convert from S16 to U8
	vnh7040_pwm = pwm;
	
		///STEP3: Compute reverse direction according to layout
	//Compute direction. false = forward. true = reverse
	//	dir	
	// 0	0	0 forward
	// 0	1	1 backward
	// 1	0	1 backward
	// 1	1	0 forward
	f_dir = ( ( (f_dir ^ LAYOUT_VNH7040_REVERSE) & MASK(index) ) != 0 );
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//switch: VNH7040 driver index
	switch (index)
	{
		case 0:
		{
			//Set INA, INB and SEL0 to select the direction of rotation of the motor
			SET_MASKED_BIT( PORTA.OUT, 0x30, ((f_dir)?(0x10):(0x20)) );
			//Set the PWM value of the right PWM channel of TCA0
			TCB0.CCMPH = vnh7040_pwm;
			
			break;
		}
		case 1:
		{
			//Set INA, INB and SEL0 to select the direction of rotation of the motor
			SET_MASKED_BIT( PORTA.OUT, 0xc0, ((f_dir)?(0x40):(0x80)) );
			//Set the PWM value of the right PWM channel of TCA0
			TCB1.CCMPH = vnh7040_pwm;
			
			break;
		}
		case 2:
		{
			//Set INA, INB and SEL0 to select the direction of rotation of the motor
			SET_MASKED_BIT( PORTB.OUT, 0x0c, ((f_dir)?(0x04):(0x08)) );
			//Set the PWM value of the right PWM channel of TCA0
			TCB2.CCMPH = vnh7040_pwm;
			
			break;
		}
		case 3:
		{
			//Set INA, INB and SEL0 to select the direction of rotation of the motor
			SET_MASKED_BIT( PORTD.OUT, 0xc0, ((f_dir)?(0x40):(0x80)) );
			//Set the PWM value of the right PWM channel of TCA0
			TCB3.CCMPH = vnh7040_pwm;
			
			break;
		}
		//Driver index does not exist
		default:
		{
			//a wrong VNH7040 index was given
			return true;
			
			break;
		}
	} //end switch: VNH7040 driver index
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false;
}	//End: set_vnh7040_pwm

