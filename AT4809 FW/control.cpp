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
//This class handles slope on multiple PWM channels
#include "ctrl_pwm.h"
//Generic S16 PID Class
#include "pid_s16.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

/****************************************************************************
**	GLOBAL VARS
****************************************************************************/

//Actual and target control system mode
Control_mode g_control_mode;
Control_mode g_control_mode_target;
//Store gains for the Speed PIDs and the Position PIDs
int16_t g_pid_spd_gain[NUM_PID_GAINS];
int16_t g_pid_pos_gain[NUM_PID_GAINS];
//Control System References
int16_t g_pwm_target[NUM_VNH7040];
int16_t g_pid_spd_target[NUM_ENC];
int32_t g_pid_pos_target[NUM_ENC];
//Slew rate limiter controller for the PWM channels
Orangebot::Ctrl_pwm g_vnh7040_pwm_ctrl;
//Each encoder has an associated PID controller
Orangebot::Pid_s16 g_vnh7040_pid[ NUM_ENC ];

/****************************************************************************
**	FUNCTION
****************************************************************************/

/***************************************************************************/
//!	@brief function
//!	init_ctrl_system | void
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
/***************************************************************************/

bool init_ctrl_system( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Counter
	uint8_t t;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Initialize control system in use
	g_control_mode			= CONTROL_STOP;
	g_control_mode_target	= CONTROL_STOP;
	//Initialize PWM Slew Rate Limiter
	g_vnh7040_pwm_ctrl = Orangebot::Ctrl_pwm( MAX_VNH7040_PWM, MAX_VNH7040_PWM_SLOPE );
	//For: each KP KI KD gain
	for (t = 0;t < NUM_PID_GAINS;t++)
	{
		//Initialize default PIDs Gains
		g_pid_spd_gain[t] = 0;
		g_pid_pos_gain[t] = 0;
	}
	//Initialize the PID dedicated to an encoder channel. Parameters will be loaded in runtime based on control method.
	//For each encoder
	for (t = 0;t < NUM_ENC;t++)
	{
		//Initialize reference
		g_pid_spd_target[ t ] = (int16_t)0;
		//Initialize the control system associated with that encoder
		g_vnh7040_pid[ t ] = Orangebot::Pid_s16();
		//Initialize Unlock handler
		g_vnh7040_pid[ t ].limit_sat_th() = PID_SAT_TH;
		//Set command limiter
		g_vnh7040_pid[ t ].limit_cmd_max() = MAX_VNH7040_PWM;
		g_vnh7040_pid[ t ].limit_cmd_min() = -MAX_VNH7040_PWM;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: init_ctrl_system | void

/***************************************************************************/
//!	@brief function
//!	control_system
/***************************************************************************/
//! @return bool | true: failed to execute control system
//! @brief select and execute correct control system
//! @details
/***************************************************************************/

bool control_system( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	CONTROL MODE
	//----------------------------------------------------------------
	
	//If: switch of control mode
	if (g_control_mode != g_control_mode_target)
	{
		//Control mode is the one desired by the user
		g_control_mode = g_control_mode_target;
		//Signal switch
		send_msg_ctrl_mode();
		//For: Each VNH7040 motor driver
		for (t= 0;t < NUM_VNH7040;t++)
		{
			//Reset the PWM controllers
			g_vnh7040_pwm_ctrl.reset();	
		}
		//Switch: control_mode
		switch(g_control_mode)
		{
			//STOP
			case Control_mode::CONTROL_STOP:
			{
				
				//do nothing
				break;
			}
			//PWM Control
			case Control_mode::CONTROL_PWM:
			{
				//do nothing
				break;
			}
			//Speed Control / Speed PID
			case Control_mode::CONTROL_FAST_SPD:
			{	
				//For: Each encoder
				for (t= 0;t < NUM_ENC;t++)
				{
					//Initialize target to stop
					g_pid_spd_target[t] = 0;
					//Apply the correct PID gain parameters
					g_vnh7040_pid[t].gain_kp() = g_pid_spd_gain[PID_GAIN_KP];
					g_vnh7040_pid[t].gain_ki() = g_pid_spd_gain[PID_GAIN_KI];
					g_vnh7040_pid[t].gain_kd() = g_pid_spd_gain[PID_GAIN_KD];
					//Reset the Speed PID controller
					g_vnh7040_pid[t].reset();
				}
				break;
			}
			//Speed Control / Position PID
			case Control_mode::CONTROL_SLOW_SPD:
			//Position Control / Position PID
			case Control_mode::CONTROL_POS:
			{
				//For: Each encoder
				for (t= 0;t < NUM_ENC;t++)
				{
					//Initialize target to current position
					g_pid_pos_target[t] = g_enc_pos[t];
					//Apply the correct PID gain parameters
					g_vnh7040_pid[t].gain_kp() = g_pid_pos_gain[PID_GAIN_KP];
					g_vnh7040_pid[t].gain_ki() = g_pid_pos_gain[PID_GAIN_KI];
					g_vnh7040_pid[t].gain_kd() = g_pid_pos_gain[PID_GAIN_KD];
					//Reset the Speed PID controller
					g_vnh7040_pid[t].reset();
				}
				break;
			}
			//Unhandled control system
			default:
			{
				report_error(Error_code::ERR_CODE_UNDEFINED_CONTROL_SYSTEM);
				return true;  //FAIL
			}
		}	//End Switch: control_mode
	}	//End If: switch of control mode
	//otherwise control mode is already correct
	else
	{
		//Do nothing
	}
	
		//----------------------------------------------------------------
		//	UPDATE ENCODER POSITION AND SPEED
		//----------------------------------------------------------------

	//Update position and speed encoder readings
	if (process_enc() == true)
	{
		//Report error to the caller
		report_error(Error_code::ERR_ENC_RETRY);
		return true;  //FAIL
	}
	
		//----------------------------------------------------------------
		//	EXECUTE CONTROL SYSTEM
		//----------------------------------------------------------------
	
	//! Execute a step in the right control mode
	//If: control system is in STOP state
	if (g_control_mode == CONTROL_STOP)
	{
		//Forcefully reset the PWM controller
		g_vnh7040_pwm_ctrl.reset();
	}	//End If: control system is in STOP state
	//If: control system is open loop PWM
	else if (g_control_mode == CONTROL_PWM)
	{
		//For: every VNH7040 motor controller
		for (t = 0;t < NUM_VNH7040;t++)
		{
			//Set PWM target
			g_vnh7040_pwm_ctrl.target(t) = g_pwm_target[t];
		} //End for: every VNH7040 motor controller		
	}	//End If: control system is open loop PWM
	//If: control system is closed loop speed and encoder speed is high enough
	else if (g_control_mode == CONTROL_FAST_SPD)
	{
		//For: every encoder
		for (t = 0;t < NUM_ENC;t++)
		{
			//Execute control system and Feed command to the PWM slew rate limiter
			g_vnh7040_pwm_ctrl.target( t ) = g_vnh7040_pid[t].exe( g_pid_spd_target[t], g_enc_spd[t] );
			//If: PID has become unlocked
			if (g_vnh7040_pid[t].get_pid_status() == true)
			{
				//Report error to the caller
				report_error(Error_code::ERR_PID_UNLOCK);
				//Control system has to be disabled
				return true; //FAIL
			}
		}
	}	//End If: control system is closed loop speed
	//If: control system is closed loop speed and encoder speed is high enough
	else if (g_control_mode == CONTROL_SLOW_SPD)
	{
		//For: every encoder
		for (t = 0;t < NUM_ENC;t++)
		{
			//Accumulate target speed inside absolute target position. Position = Speed integral
			g_pid_pos_target[t] += g_pid_spd_target[t];
			//Execute control system with position PID parameters
			g_vnh7040_pwm_ctrl.target( t ) = g_vnh7040_pid[t].exe( g_pid_pos_target[t], g_enc_pos[t] );
			//If: PID has become unlocked
			if (g_vnh7040_pid[t].get_pid_status() == true)
			{
				//Report error to the caller
				report_error(Error_code::ERR_PID_UNLOCK);
				//Control system has to be disabled
				return true; //FAIL
			}
		}	//End For: every encoder
	}	//End If: control system is closed loop speed
	//If: control system is closed loop speed and encoder speed is high enough
	else if (g_control_mode == CONTROL_POS)
	{
		//For: every encoder
		for (t = 0;t < NUM_ENC;t++)
		{
			//Execute control system with position PID parameters
			g_vnh7040_pwm_ctrl.target( t ) = g_vnh7040_pid[t].exe( g_pid_pos_target[t], g_enc_pos[t] );
			//If: PID has become unlocked
			if (g_vnh7040_pid[t].get_pid_status() == true)
			{
				//Report error to the caller
				report_error(Error_code::ERR_PID_UNLOCK);
				//Control system has to be disabled
				return true; //FAIL
			}
		}	//End For: every encoder
	}	//End If: control system is closed loop speed
	//if: undefined control system
	else
	{
		//Forcefully reset the PWM controller
		g_vnh7040_pwm_ctrl.reset();
		//Signal error
		report_error( ERR_CODE_UNDEFINED_CONTROL_SYSTEM );
		//Control system has to be disabled
		return true; //FAIL
	}	//End if: undefined control system

	//----------------------------------------------------------------
	//	Apply PWM Setting to VNH7040 Drivers
	//----------------------------------------------------------------

	//Execute slew rate limiter
	g_vnh7040_pwm_ctrl.update();
	//For: every VNH7040 motor controller
	for (t = 0;t < NUM_VNH7040;t++)
	{
		//Apply computed PWM settings to the motors
		set_vnh7040_pwm( t, g_vnh7040_pwm_ctrl.pwm(t) );
	} //End for: every VNH7040 motor controller

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: control_system | void

/***************************************************************************/
//!	@brief function
//!	set_spd_pid | int16_t, int16_t, int16_t
/***************************************************************************/
//! @param gain_kp | proportional gain
//! @param gain_ki | integral gain
//! @param gain_kd | derivative gain
//! @return bool | always succeed
//! @details
//! Set the Speed PID target gain for all encoder channels and force control system to reload
/***************************************************************************/

bool set_spd_pid( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd )
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

	//By setting to STOP, control system will be forced to reload
	g_control_mode = Control_mode::CONTROL_STOP;
	//Save the new gains. They will be loaded next change of control system
	g_pid_spd_gain[PID_GAIN_KP] = gain_kp;
	g_pid_spd_gain[PID_GAIN_KI] = gain_ki;
	g_pid_spd_gain[PID_GAIN_KD] = gain_kd;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_spd_pid | int16_t, int16_t, int16_t

/***************************************************************************/
//!	@brief function
//!	set_pos_pid | int16_t, int16_t, int16_t
/***************************************************************************/
//! @param gain_kp | proportional gain
//! @param gain_ki | integral gain
//! @param gain_kd | derivative gain
//! @return bool | always succeed
//! @details
//! Set the position PID target gain for all encoder channels and force control system to reload
/***************************************************************************/

bool set_pos_pid( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd )
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

	//By setting to STOP, control system will be forced to reload
	g_control_mode = Control_mode::CONTROL_STOP;
	//Save the new gains. They will be loaded next change of control system
	g_pid_pos_gain[PID_GAIN_KP] = gain_kp;
	g_pid_pos_gain[PID_GAIN_KI] = gain_ki;
	g_pid_pos_gain[PID_GAIN_KD] = gain_kd;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_pos_pid | int16_t, int16_t, int16_t