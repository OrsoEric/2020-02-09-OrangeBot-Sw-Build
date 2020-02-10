/****************************************************************************
**	OrangeBot Project
*****************************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************************
**	PID S16
*****************************************************************************
**	@author		Orso Eric
**	@version	2020-02-07
**	Compiler Flags:
****************************************************************************/

/****************************************************************************
**	HYSTORY VERSION
*****************************************************************************
**		2019-11-08
**	Pid_s16 C++ class meant for use in microcontroller.
**		2020-01-27
**	Atmel C++ compiler had trouble with AT_FP_SAT_MUL16. Updated it
**		2020-01-28
**	Update to V2
**	Remove AT_FP_SAT_MUL16 macros. Atmel C++ seems to have provlem with gain
**	Also, using a 32bit intermediate register might be faster than refactor down to 16b after each MAC
**	Added error accessible from user. This gives an idea of how hard a time the PID is having
**	Added error LP filter
**	Added .csv output file on open office calc to see nice charts on test bench
**	Added overload to exe that allow to process 32b feedback and 32b target
**	Slew Rate Protection. If PID becomes unstable, the slew rate of the command shot up.
**	Objective of protections is to detect dangerous conditions: Command pegged +/-100% Command oscillating
**		2020-01-29
**	Tested slew rate protection for both 16b and 32b execution units
**		2020-02-04
**	Two slopes linear error gain. At low error, increase gain. At high error, decrease gain.
**	It's meant to increase response at low error and decrease instability at high error
**	Works best when used over a very small input range
**	Moved to SSHR_RTO macro to add rounding to division
**	Add three sample derivative.
**		2020-02-07
**	Moved define into an enum on the orangebot namespace
**	Completed exe16b method
**	Fixed problem with integrator
**		2020-02-10
**	Stack overflow help for cast
**	https://stackoverflow.com/questions/60124970/32b-multiplication-in-8b-uc-with-avr-g-vs-32b-multiplication-on-x86-with-gcc
****************************************************************************/

/****************************************************************************
**	DESCRIPTION
*****************************************************************************
**		TWO SLOPE ERROR GAIN
**
**	At low absolute error, the gain is boosted. It's meant to decrease error at low error without having to boost the KP
**	At high absolute error, the gain is dampened. It's meant to decrease instability at high error
**				^OUT
**              |         *
**              |     *
**       OUT_TH>| *
**              |*
**------------------------------>IN
**             *| ^
**            * | IN_TH
**        *     |
**    *         |
**
*****************************************************************************
**		INTEGRATOR SATURATION
**
**  When command is saturated, accumulator is forbidden to increase in magnitude
**	This decreases ringing by reducing the time needed to unload the accumulator
**
*****************************************************************************
**		THREE SAMPLE ASYMMETRIC DERIVATIVE
**
**	A more accurate discrete time derivative that also avoids ringing artefacts
**	The cost is a longer status vector and some more computation
**
*****************************************************************************
**		ERROR AND SLEW RATE MONITORING
**
**	Input error and command slew rate are computed and passed through an optional
**	low pass filter before reporting.
**	Error and Slew Rate allows the user to estimate how hard a time the PID is
**	having in maintaining the reference
**
*****************************************************************************
**		PID CUTOFF PROTECTION
**
**	The PID automatically cuts off by detecting two safety hazards:
**	1) command has been pegged to its limit for too long
**	2) command slew rate has been above a threshold for too long
**	This allow detection of disconnected feedback sensor, inversion of gain
**	and ringing. The PID must be reset by the user to resume operation.
**
*****************************************************************************
**		ACCUMULATOR DECAY (NOT INSTALLED)
**
**	The accumulator decays by a set fraction each update
**	This allows the accumulator to go to zero when error is low instead of approaching a constant
**	Allowing the proportional component to shoulder the bulk of the correction in steady state
**	Decay does not work. A way to prevent decay when there is an error must be installed.
****************************************************************************/

/****************************************************************************
**	KNOWN BUG
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	INCLUDES
****************************************************************************/

#include <stdint.h>
//Atmel utility macros
#include "at_utils.h"
//Debug
//#define ENABLE_DEBUG
#include "debug.h"
//Class Header
#include "Pid_s16.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

namespace Orangebot
{

/****************************************************************************
**	GLOBAL VARIABILES
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	CONSTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Constructor
//!	Pid_s16 | void
/***************************************************************************/
//! @return no return
//!	@details
//! Empty constructor
/***************************************************************************/

Pid_s16::Pid_s16( void )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Initialize class vars
	this -> init();
	//Reset PID class
	this -> reset();

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();

	return;	//OK
}	//end constructor:

/****************************************************************************
*****************************************************************************
**	DESTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Destructor
//!	Pid_s16 | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Empty destructor
/***************************************************************************/

Pid_s16::~Pid_s16( void )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();

	return;	//OK
}	//end destructor: Empty Destructor

/****************************************************************************
*****************************************************************************
**	OPERATORS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	SETTERS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	set_limit_cmd | int16_t, int16_t
/***************************************************************************/
//! @param min |
//! @param max |
//! @return no return
//!	@details
//! Set command limits
/***************************************************************************/

bool Pid_s16::set_limit_cmd( int16_t min, int16_t max )
{
	//Trace Enter
	DENTER_ARG("min: %d | max: %d\n", min, max);

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//If: bad limits
	if (max <= min)
	{
		DRETURN_ARG("ERR: max is not above min!");
		return true;	//FAIL
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	this -> g_cmd_min = min;
	this -> g_cmd_max = max;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return false;	//OK
}	//end method: set_limit_cmd | int16_t, int16_t

/****************************************************************************
*****************************************************************************
**	GETTERS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	get_speed_status | void
/***************************************************************************/
//! @return false = OK | true = protection
//!	@details
//! PID Status. false = OK | true = protection
/***************************************************************************/

bool Pid_s16::get_pid_status( void )
{
	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	return this -> g_b_unlock;
}	//end method: get_speed_status | void

/***************************************************************************/
//!	@brief Getter
//!	get_err | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Error committed by the PID
/***************************************************************************/

int16_t Pid_s16::get_err( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_err;
}	//end reference: get_err | void

/***************************************************************************/
//!	@brief Getter
//!	get_slew_rate | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Slew rate of the generated command. Give an indication of how hard a time the PID is having keeping a lock
/***************************************************************************/

int16_t Pid_s16::get_slew_rate( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_slew_rate;
}	//end reference: get_slew_rate | void

/****************************************************************************
*****************************************************************************
**	REFERENCES
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Reference Operator
//!	limit_cmd_max | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

int16_t &Pid_s16::limit_cmd_max( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_cmd_max;
}	//end reference: limit_cmd_max | void

/***************************************************************************/
//!	@brief Reference Operator
//!	limit_cmd_min | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

int16_t &Pid_s16::limit_cmd_min( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_cmd_min;
}	//end reference: limit_cmd_min | void

/***************************************************************************/
//!	@brief Reference Operator
//!	limit_sat_th | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

uint16_t &Pid_s16::limit_sat_th( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_sat_th;
}	//end reference: limit_sat_th | void

/***************************************************************************/
//!	@brief Reference Operator
//!	gain_kp | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

int16_t &Pid_s16::gain_kp( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_kp;
}	//end reference: gain_kp | void

/***************************************************************************/
//!	@brief Reference Operator
//!	gain_kd | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

int16_t &Pid_s16::gain_kd( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_kd;
}	//end reference: gain_kd | void

/***************************************************************************/
//!	@brief Reference Operator
//!	gain_ki | void
/***************************************************************************/
//! @return int16_t |
//!	@details
//!	Set the gain of the gain of the PID
/***************************************************************************/

int16_t &Pid_s16::gain_ki( void )
{
	//--------------------------------------------------------------------------
	//	RETURN
	//--------------------------------------------------------------------------

	return this -> g_ki;
}	//end reference: gain_ki | void

/****************************************************************************
*****************************************************************************
**	TESTERS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	PUBLIC METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	exe | int16_t, int16_t
/***************************************************************************/
//! @param reference | target output of the system to be controlled
//! @param feedback | current output of the system to be controlled
//! @return no return
//!	@details
//! Execute a step of the PID controller
/***************************************************************************/

int16_t Pid_s16::exe( int16_t reference, int16_t feedback )
{
	//Trace Enter
	DENTER_ARG("16b | reference: %d, feedback: %d\n", reference, feedback);

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//error
	int16_t err;
	//high resolution command
	int32_t cmd32 = 0;
	//Output command
	int16_t cmd16;
	//Detect saturation of command
	bool f_sat;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//If PID is in protection
	if (this -> g_b_unlock == true)
	{
		//FAIL: PID is in protection and requires reset to operate again
		DRETURN_ARG("ERR: PID is unlocked\n");
		return (int16_t)0;
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------
	//		ALGORITHM:
	//	Compute error

		///--------------------------------------------------------------------------
		///	COMPUTE ERROR
		///--------------------------------------------------------------------------

	//Compute saturated16b error
	err = AT_SAT_SUM( reference, -feedback, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
	DPRINT("ERR: %d", err);

		///--------------------------------------------------------------------------
		///	TWO SLOPES ERROR GAIN
		///--------------------------------------------------------------------------
		//	Two slopes linear error gain.
		//	X < -XTH
		//		Y = -YTH +(X +XTH) *(YMAX -YTH) / (XMAX -XTH)
		//	X >= -XTH && x <= XTH
		//		Y = X *YTH /XTH
		//	X > XTH
		//		Y = YTH +(X- XTH) *(YMAX -YTH) / (XMAX -XTH)

	//If two slope correction is enabled
    if (Pid_s16_config::TWO_SLOPE_INPUT_TH != Pid_s16_config::TWO_SLOPE_OUTPUT_TH)
    {
		//If: Negative low gain region
		if (err < -Pid_s16_config::TWO_SLOPE_INPUT_TH)
		{
			//Compute gain correction
			int16_t tmp = SSHR_RTO(err +Pid_s16_config::TWO_SLOPE_INPUT_TH, Pid_s16_config::TWO_SLOPE_GAIN_FP);
			//Compute offset
			err = AT_SAT_SUM( -Pid_s16_config::TWO_SLOPE_OUTPUT_TH, tmp, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		}
		//If: Positive low gain region
		else if (err > Pid_s16_config::TWO_SLOPE_INPUT_TH)
		{
			//Compute gain correction
			int16_t tmp = SSHR_RTO(err -Pid_s16_config::TWO_SLOPE_INPUT_TH, Pid_s16_config::TWO_SLOPE_GAIN_FP);
			//Compute offset
			err = AT_SAT_SUM( +Pid_s16_config::TWO_SLOPE_OUTPUT_TH, tmp, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		}
		//if: High gain region
		else //if ((err32 >= -Pid_s16_config::TWO_SLOPE_INPUT_TH) && (err32 <= Pid_s16_config::TWO_SLOPE_INPUT_TH))
		{
			err *= (Pid_s16_config::TWO_SLOPE_OUTPUT_TH /Pid_s16_config::TWO_SLOPE_INPUT_TH);
		}
    }
	DPRINT_NOTAB(" | Two slope error correction: %d\n", err);

		///--------------------------------------------------------------------------
		///	PID ERROR USER REPORT
		///--------------------------------------------------------------------------
		//	Apply an optional filter to the error and report it to the user of this PID class
		//	Error gives an indication of how hard a time the PID is having

		//! PID error accessible from user side
	//If: LP filter is disabled
	if ((1<<Pid_s16_config::LP_ERR_FP) == Pid_s16_config::LP_ERR_GAIN)
	{
		//Save the error. The user might want to know how is the PID doing
		this -> g_err = err;
	}
	//If: LP filter is enabled
	else
	{
		//Promote error to 32b
		int32_t err32 = err;
		//Promote old error to 32b
		int32_t old_err32 = this -> g_err;
		//Compute LP filter on error
		err32 = err32 *Pid_s16_config::LP_ERR_GAIN +old_err32 *((1<<Pid_s16_config::LP_ERR_FP) -Pid_s16_config::LP_ERR_GAIN);
		//Apply fixed point normalization
		err32 = SSHR_RTO( err32, Pid_s16_config::LP_ERR_FP );
		//Apply saturation
		err32 = AT_SAT( err32, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		//Save filtered error on 16b
		this -> g_err = err32;
	}

		///--------------------------------------------------------------------------
		///	COMPUTE FEED FORWARD
		///--------------------------------------------------------------------------
		//	The Feed Forward component directly translates change in target to change in output
		//	This allows the output to respond faster to changes in target

		///--------------------------------------------------------------------------
		///	COMPUTE PROPORTIONAL
		///--------------------------------------------------------------------------
		//	The proportional component performs the bulk of the quick command generation.
		//	The higher the error, the higher the command has to be to bring the error down

	//Fetch gain
	int16_t gain_kp = this -> g_kp;
	//if: gain is active
	if (gain_kp != 0)
	{
		//Promote error to 32b
		int32_t err32 = err;
		//MAC proportional error contribution
		cmd32 = err32 *gain_kp;
		DPRINT("proportional command: %d\n", err32 *gain_kp /(1<<Pid_s16_config::GAIN_FP));
	}

		///--------------------------------------------------------------------------
		///	COMPUTE INTEGRATIVE
		///--------------------------------------------------------------------------
		//	The integral component allows the PID to eventually achieve zero error
		//	Bigger integral means quicker convergence, but also more undershoot and overshoot
		//	Increasing the integral component carry the biggest risk of resonance, like in a mike/speaker feedback

	//Fetch gain
	int16_t gain_ki = this -> g_ki;
	//Fetch accumulator
	int16_t acc = this -> g_acc;
	//if: gain is active
	if (gain_ki != 0)
	{
		//Promote accumulator to 32b
		int32_t acc32 = err +acc;
		//Saturate
		acc32 = AT_SAT( acc32, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		//MAC integral error
		cmd32 += acc32 *gain_ki;
		DPRINT("integral command: %d\n", acc32 *gain_ki /(1<<Pid_s16_config::GAIN_FP));
		//Update local 16b accumulator for later
		acc = acc32;
	}

		///--------------------------------------------------------------------------
		///	COMPUTE DERIVATIVE
		///--------------------------------------------------------------------------
		//	The derivative component is meant to keep the integral component in check
		//	This implementation use a simple two point derivative. I have a vastly superior four point derivative that minimize error to natural system response.

	//Fetch gain
	int16_t gain_kd = this -> g_kd;
	//if: gain is active
	if (gain_kd != 0)
	{
		//Three samples asymmetric discrete derivative
		int32_t err_d32 = Pid_s16_config::DERIVATIVE_PARAM0 *err + Pid_s16_config::DERIVATIVE_PARAM1 *this -> g_old_err[ 0 ]+ Pid_s16_config::DERIVATIVE_PARAM2 *this -> g_old_err[ 1 ];
		err_d32 = SSHR_RTO( err_d32, Pid_s16_config::DERIVATIVE_PARAM_FP );
		//MAC derivative
		cmd32 += err_d32 *gain_kd;
		DPRINT("derivative error: %d | derivative command: %d\n", err_d32, err_d32 *gain_kd /(1<<Pid_s16_config::GAIN_FP));
	}

		///--------------------------------------------------------------------------
		///	COMPUTE COMMAND
		///--------------------------------------------------------------------------
		//	Command is renormalized as the gais are expressed in fixed point

	//Apply Fixed Point renormalization
	cmd32 = SSHR_RTO( cmd32, Pid_s16_config::GAIN_FP );
	//Compute saturation of the command
	cmd32 = AT_SAT( cmd32, this -> g_cmd_max, this -> g_cmd_min );
	//Compute the command
	cmd16 = (int16_t)0 +cmd32;
	DPRINT("cmd32: %d | cmd16: %d\n", cmd32, cmd16 );

		///--------------------------------------------------------------------------
		///	COMPUTE COMMAND SLEW RATE
		///--------------------------------------------------------------------------
		//	Slew Rate Protection. If PID becomes unstable, the slew rate of the command shot up.

	//Compute command derivative
	int16_t sr_tmp = cmd16 -this -> g_old_cmd;
	//Without sign
	sr_tmp = AT_ABS( sr_tmp );

		//! Compute Slew rate
	//If: LP filter is disabled
	if ((1<<Pid_s16_config::LP_SLEW_RATE_FP) == Pid_s16_config::LP_SLEW_RATE_GAIN)
	{
		//Save the error. The user might want to know how is the PID doing
		this -> g_slew_rate = sr_tmp;
	}
	//If: LP filter is enabled
	else
	{
		//Fetch slew rate
		int16_t slew_rate16 = this -> g_slew_rate;
		//Promote slew rate to 32b
		int32_t slew_rate32 = slew_rate16;
		//Compute LP filter on error
		slew_rate32 = sr_tmp *Pid_s16_config::LP_SLEW_RATE_GAIN +slew_rate32 *((1<<Pid_s16_config::LP_SLEW_RATE_FP) -Pid_s16_config::LP_SLEW_RATE_GAIN);
		//Normalize
		slew_rate32 = SSHR_RTO( slew_rate32, Pid_s16_config::LP_SLEW_RATE_FP );
		//Saturate
		slew_rate32 = AT_SAT( slew_rate32, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		//Demote
		slew_rate16 = slew_rate32;
		//Write back slew rate
		this -> g_slew_rate = slew_rate16;
	}	//End If: LP filter is enabled

	//Detect slew rate limit exceeded flag
	bool f_sr = ( this -> g_slew_rate > ((this -> g_cmd_max / (1<<Pid_s16_config::SLEW_RATE_LIMIT)) -(this -> g_cmd_min / (1<<Pid_s16_config::SLEW_RATE_LIMIT))) );
	DPRINT("slew rate: %d %d | Slew rare treshold: %d | Slew rate limit detected: %d\n", sr_tmp, this -> g_slew_rate, ((this -> g_cmd_max / (1<<Pid_s16_config::SLEW_RATE_LIMIT)) -(this -> g_cmd_min / (1<<Pid_s16_config::SLEW_RATE_LIMIT))), f_sr );

		///--------------------------------------------------------------------------
		///	SATURATION DETECTION
		///--------------------------------------------------------------------------
		//	This feature is meant to detect when command is saturated for too long.
		//	This means that the PID is unable to keep up with the system and the PID is unlocked
		//	Es. A motor as encoder connected in reverse. The PID can never bring error to zero, so cut off to prevent damage.
		//	Es. Achieve a given speed against too great a resistive force. Command cannot be achieved with this level of power and PID cut off.

	//Detect command saturation
	f_sat = ((cmd16 >= this -> g_cmd_max) || (cmd16 <= this -> g_cmd_min));

	//If feature is disabled
	if (this -> g_sat_th == 0)
	{
		//do nothing
	}
	//If saturation detected or if slew rate limit has been exceeded
	else if ((f_sat == true) || (f_sr == true))
	{
		//Increment counter
		this -> g_sat_cnt++;
		DPRINT("Saturation detected %d. Slew rate limit detected %d | %d of %d\n", f_sat, f_sr, this -> g_sat_cnt, this -> g_sat_th);
		//Detect error condition
		if (this -> g_sat_cnt > this -> g_sat_th)
		{
			//Signal that the PID is now in protection mode
			this -> g_b_unlock = true;
			//FAIL: PID is in protection and requires reset to operate again
			DRETURN_ARG("ERR: Command has been saturated for too long. Unlock.\n");
			return (int16_t)0;
		}
	}
	//If saturation not detected
	else
	{
		DPRINT("Reset saturation counter from %d to 0\n", this -> g_sat_cnt);
		//Clear the counter. Short saturations are permitted as the PID can still somehow keep up
		this -> g_sat_cnt = 0;
	}

		///--------------------------------------------------------------------------
		///	UPDATE REGISTERS
		///--------------------------------------------------------------------------

	//Update command memory register
	this -> g_old_cmd = cmd16;
	//Update derivative shift register
	this -> g_old_err[1] = this -> g_old_err[0];
	this -> g_old_err[0] = err;
	//Authorize update of integrative register only if update would push a reduction of the command from saturation
	if ((f_sat == false) || (AT_ABS(acc) < AT_ABS(this -> g_acc)))
	{
		DPRINT("update integral accumulator: %d -> %d\n", this -> g_acc, acc);
		//Update accumulator register
		this -> g_acc = acc;
	}
	else
	{
		DPRINT("Saturation detected. Update of accumulator not authorized\n");
	}

	DPRINT("old: %5d %5d, acc: %5d\n", this -> g_old_err[1] , this -> g_old_err[0], this -> g_acc);

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN_ARG("command: %d\n", cmd16);
	return cmd16;
}	//end method: exe | int16_t, int16_t

/***************************************************************************/
//!	@brief Public Method
//!	exe | int32_t, int32_t
/***************************************************************************/
//! @param reference | target output of the system to be controlled
//! @param feedback | current output of the system to be controlled
//! @return no return
//!	@details
//! Execute a step of the PID controller
//! @bug 2020-02-02a
//! Atmel C++ compiler cannot execute int32_t = int16_t *int16_t
//!	Always promote first operand of multiplication to 32b
//! int32_t = int32_t *int16_t
/***************************************************************************/

int16_t Pid_s16::exe( int32_t reference, int32_t feedback )
{
	//Trace Enter
	DENTER_ARG("32b | reference: %d, feedback: %d\n", reference, feedback);

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//error
	int32_t err32;
	int16_t err;
	//high resolution command
	int32_t cmd32 = 0;
	//Output command
	int16_t cmd16;
	//Detect saturation of command
	bool f_sat;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//If PID is in protection
	if (this -> g_b_unlock == true)
	{
		//FAIL: PID is in protection and requires reset to operate again
		DRETURN_ARG("ERR: PID is unlocked\n");
		return (int16_t)0;
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------
	//		ALGORITHM:
	//	Compute error

		///--------------------------------------------------------------------------
		///	COMPUTE ERROR
		///--------------------------------------------------------------------------

	//Compute 32b error
	err32 = reference -feedback;
	DPRINT("ERR: %d", err32);

		///--------------------------------------------------------------------------
		///	TWO SLOPES ERROR GAIN
		///--------------------------------------------------------------------------
		//	X < -XTH
		//		Y = -YTH +(X +XTH) *(YMAX -YTH) / (XMAX -XTH)
		//	X >= -XTH && x <= XTH
		//		Y = X *YTH /XTH
		//	X > XTH
		//		Y = YTH +(X- XTH) *(YMAX -YTH) / (XMAX -XTH)

	//If two slope correction is enabled
    if (Pid_s16_config::TWO_SLOPE_INPUT_TH != Pid_s16_config::TWO_SLOPE_OUTPUT_TH)
    {
		//If: Negative low gain region
		if (err32 < -Pid_s16_config::TWO_SLOPE_INPUT_TH)
		{
			//High resolution correction with no rounding. Very expensive and with aliasing artefacts.
			//err32 = -Pid_s16_config::TWO_SLOPE_OUTPUT_TH +(err32 +Pid_s16_config::TWO_SLOPE_INPUT_TH) *(32767 -Pid_s16_config::TWO_SLOPE_OUTPUT_TH) / (32767 -Pid_s16_config::TWO_SLOPE_INPUT_TH);
			//Low aliasing correction that won't use the full error range
			err32 = -Pid_s16_config::TWO_SLOPE_OUTPUT_TH +SSHR_RTO(err32 +Pid_s16_config::TWO_SLOPE_INPUT_TH, Pid_s16_config::TWO_SLOPE_GAIN_FP);
		}
		//If: Positive low gain region
		else if (err32 > Pid_s16_config::TWO_SLOPE_INPUT_TH)
		{
			//High resolution correction with no rounding. Very expensive and with aliasing artefacts.
			//err32 = +Pid_s16_config::TWO_SLOPE_OUTPUT_TH +(err32 -Pid_s16_config::TWO_SLOPE_INPUT_TH) *(32767 -Pid_s16_config::TWO_SLOPE_OUTPUT_TH) / (32767 -Pid_s16_config::TWO_SLOPE_INPUT_TH);
			//Low aliasing correction that won't use the full error range
			err32 = +Pid_s16_config::TWO_SLOPE_OUTPUT_TH +SSHR_RTO(err32 -Pid_s16_config::TWO_SLOPE_INPUT_TH, Pid_s16_config::TWO_SLOPE_GAIN_FP);
		}
		//if: High gain region
		else //if ((err32 >= -Pid_s16_config::TWO_SLOPE_INPUT_TH) && (err32 <= Pid_s16_config::TWO_SLOPE_INPUT_TH))
		{
			err32 = err32 *(Pid_s16_config::TWO_SLOPE_OUTPUT_TH /Pid_s16_config::TWO_SLOPE_INPUT_TH);
		}
    }
	DPRINT_NOTAB(" | Two slope error correction: %d", err32);
	//Saturate error to 16b limits
	err32 = AT_SAT( err32, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
    //Assign to 16b error
	err = (int16_t)0 +err32;
	DPRINT_NOTAB(" | ERR: %d\n", err);

		///--------------------------------------------------------------------------
		///	PID ERROR USER REPORT
		///--------------------------------------------------------------------------
		//	Apply an optional filter to the error and report it to the user of this PID class
		//	Error gives an indication of how hard a time the PID is having

		//! PID error accessible from user side
	//If: LP filter is disabled
	if ((1<<Pid_s16_config::LP_ERR_FP) == Pid_s16_config::LP_ERR_GAIN)
	{
		//Save the error. The user might want to know how is the PID doing
		this -> g_err = err;
	}
	//If: LP filter is enabled
	else
	{
		//Promote to 32b
		int32_t old_err32 = this -> g_err;
		//Compute LP filter on error
		err32 = err32 *Pid_s16_config::LP_ERR_GAIN +old_err32 *((1<<Pid_s16_config::LP_ERR_FP) -Pid_s16_config::LP_ERR_GAIN);
		//Apply fixed point normalization
		err32 = SSHR_RTO( err32, Pid_s16_config::LP_ERR_FP );
		//SSHR32_RTO( err32, PID_ERR_FP );
		//Apply saturation
		err32 = AT_SAT( err32, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		//Save filtered error on 16b
		this -> g_err = err32;
	}

		///--------------------------------------------------------------------------
		///	COMPUTE FEED FORWARD
		///--------------------------------------------------------------------------
		//	The Feed Forward component directly translates change in target to change in output
		//	This allows the output to respond faster to changes in target

		///--------------------------------------------------------------------------
		///	COMPUTE PROPORTIONAL
		///--------------------------------------------------------------------------
		//	The proportional component performs the bulk of the quick command generation.
		//	The higher the error, the higher the command has to be to bring the error down

	//if: gain is active
	if (this -> g_kp != 0)
	{
		//Compute proportional command. Promote operand to execute operation at 32b resolution
		cmd32 = (int32_t)err32 *this -> g_kp;
		DPRINT("proportional command: %d\n", err32 *gain_kp /(1<<Pid_s16_config::GAIN_FP));
	}

		///--------------------------------------------------------------------------
		///	COMPUTE INTEGRATIVE
		///--------------------------------------------------------------------------
		//	The integral component allows the PID to eventually achieve zero error
		//	Bigger integral means quicker convergence, but also more undershoot and overshoot
		//	Increasing the integral component carry the biggest risk of resonance, like in a mike/speaker feedback

	//Fetch accumulator
	int16_t acc = this -> g_acc;
	//if: gain is active
	if (this -> g_ki != 0)
	{
		//Integrate error inside the accumulator
		acc = AT_SAT_SUM( err, acc, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		//MAC integral error. Promote operand to execute operation at 32b resolution
		cmd32 += (int32_t)acc *this -> g_ki;
		DPRINT("integral command: %d\n", acc32 *gain_ki /(1<<Pid_s16_config::GAIN_FP));
	}

		///--------------------------------------------------------------------------
		///	COMPUTE DERIVATIVE
		///--------------------------------------------------------------------------
		//	The derivative component is meant to keep the integral component in check
		//	This implementation use a simple two point derivative. I have a vastly superior four point derivative that minimize error to natural system response.

	//if: gain is active
	if (this -> g_kd != 0)
	{		
		//Three samples asymmetric discrete derivative
		int32_t err_d32 = (int32_t)err *Pid_s16_config::DERIVATIVE_PARAM0 + Pid_s16_config::DERIVATIVE_PARAM1 *this -> g_old_err[ 0 ]+ Pid_s16_config::DERIVATIVE_PARAM2 *this -> g_old_err[ 1 ];
		err_d32 = SSHR_RTO( err_d32, Pid_s16_config::DERIVATIVE_PARAM_FP );
		//MAC derivative
		cmd32 += err_d32 *this -> g_kd;
		DPRINT("derivative error: %d | derivative command: %d\n", err_d32, err_d32 *gain_kd /(1<<Pid_s16_config::GAIN_FP));
	}

		///--------------------------------------------------------------------------
		///	COMPUTE COMMAND
		///--------------------------------------------------------------------------
		//	Command is renormalized as the gains are expressed in fixed point

	//Apply Fixed Point renormalization
	cmd32 = SSHR_RTO( cmd32, Pid_s16_config::GAIN_FP );
	//Compute saturation of the command
	cmd32 = AT_SAT( cmd32, this -> g_cmd_max, this -> g_cmd_min );
	//Compute the command
	cmd16 = (int16_t)0 +cmd32;
	DPRINT("cmd32: %d | cmd16: %d\n", cmd32, cmd16 );

		///--------------------------------------------------------------------------
		///	COMPUTE COMMAND SLEW RATE
		///--------------------------------------------------------------------------
		//	Slew Rate Protection. If PID becomes unstable, the slew rate of the command shot up.

	//Compute command derivative
	int16_t sr_tmp = cmd16 -this -> g_old_cmd;
	//Without sign
	sr_tmp = AT_ABS( sr_tmp );

		//! Compute Slew rate
	//If: LP filter is disabled
	if ((1<<Pid_s16_config::LP_SLEW_RATE_FP) == Pid_s16_config::LP_SLEW_RATE_GAIN)
	{
		//Save the error. The user might want to know how is the PID doing
		this -> g_slew_rate = sr_tmp;
	}
	//If: LP filter is enabled
	else
	{
		//Fetch slew rate
		int16_t slew_rate16 = this -> g_slew_rate;
		//Promote slew rate to 32b
		int32_t slew_rate32 = slew_rate16;
		//Compute LP filter on error
		slew_rate32 = sr_tmp *Pid_s16_config::LP_SLEW_RATE_GAIN +slew_rate32 *((1<<Pid_s16_config::LP_SLEW_RATE_FP) -Pid_s16_config::LP_SLEW_RATE_GAIN);
		//Normalize
		slew_rate32 = SSHR_RTO( slew_rate32, Pid_s16_config::LP_SLEW_RATE_FP );
		//Saturate
		slew_rate32 = AT_SAT( slew_rate32, Pid_s16_config::MAX_S16, Pid_s16_config::MIN_S16 );
		//Demote
		slew_rate16 = slew_rate32;
		//Write back slew rate
		this -> g_slew_rate = slew_rate16;
	}	//End If: LP filter is enabled

	//Detect slew rate limit exceeded flag
	bool f_sr = ( this -> g_slew_rate > ((this -> g_cmd_max / (1<<Pid_s16_config::SLEW_RATE_LIMIT)) -(this -> g_cmd_min / (1<<Pid_s16_config::SLEW_RATE_LIMIT))) );
	DPRINT("slew rate: %d %d | Slew rare treshold: %d | Slew rate limit detected: %d\n", sr_tmp, this -> g_slew_rate, ((this -> g_cmd_max / (1<<Pid_s16_config::SLEW_RATE_LIMIT)) -(this -> g_cmd_min / (1<<Pid_s16_config::SLEW_RATE_LIMIT))), f_sr );

		///--------------------------------------------------------------------------
		///	SATURATION DETECTION
		///--------------------------------------------------------------------------
		//	This feature is meant to detect when command is saturated for too long.
		//	This means that the PID is unable to keep up with the system and the PID is unlocked
		//	Es. A motor as encoder connected in reverse. The PID can never bring error to zero, so cut off to prevent damage.
		//	Es. Achieve a given speed against too great a resistive force. Command cannot be achieved with this level of power and PID cut off.

	//Detect command saturation
	f_sat = ((cmd16 >= this -> g_cmd_max) || (cmd16 <= this -> g_cmd_min));

	//If feature is disabled
	if (this -> g_sat_th == 0)
	{
		//do nothing
	}
	//If saturation detected or if slew rate limit has been exceeded
	else if ((f_sat == true) || (f_sr == true))
	{
		//Increment counter
		this -> g_sat_cnt++;
		DPRINT("Saturation detected %d. Slew rate limit detected %d | %d of %d\n", f_sat, f_sr, this -> g_sat_cnt, this -> g_sat_th);
		//Detect error condition
		if (this -> g_sat_cnt > this -> g_sat_th)
		{
			//Signal that the PID is now in protection mode
			this -> g_b_unlock = true;
			//FAIL: PID is in protection and requires reset to operate again
			DRETURN_ARG("ERR: Command has been saturated for too long. Unlock.\n");
			return (int16_t)0;
		}
	}
	//If saturation not detected
	else
	{
		DPRINT("Reset saturation counter from %d to 0\n", this -> g_sat_cnt);
		//Clear the counter. Short saturations are permitted as the PID can still somehow keep up
		this -> g_sat_cnt = 0;
	}

		///--------------------------------------------------------------------------
		///	UPDATE REGISTERS
		///--------------------------------------------------------------------------

	//Update command memory register
	this -> g_old_cmd = cmd16;
	//Update derivative shift register
	this -> g_old_err[1] = this -> g_old_err[0];
	this -> g_old_err[0] = err;
	//Authorize update of integrative register only if update would push a reduction of the command from saturation
	if ((f_sat == false) || (AT_ABS(acc) < AT_ABS(this -> g_acc)))
	{
		DPRINT("update integral accumulator: %d -> %d\n", this -> g_acc, acc);
		//Update accumulator register
		this -> g_acc = acc;
	}
	else
	{
		DPRINT("Saturation detected. Update of accumulator not authorized\n");
	}

	DPRINT("old: %5d %5d, acc: %5d\n", this -> g_old_err[1] , this -> g_old_err[0], this -> g_acc);

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN_ARG("command: %d\n", cmd16);
	return cmd16;
}	//end method: exe | int32_t, int32_t

/***************************************************************************/
//!	@brief Public Method
//!	reset | void
/***************************************************************************/
//! @return no return
//!	@details
//! Reset the PID
//! Initialize runtime vars
/***************************************************************************/

void Pid_s16::reset( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//Counter
	uint8_t t;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Reset PID status vars
	this -> g_acc		= (int16_t)0;
	//For: old error memory
	for (t = 0;t < Pid_s16_config::ERR_MEM_SIZE;t++)
	{
		this -> g_old_err[t]= (int16_t)0;
	}
	this -> g_sat_cnt	= (uint16_t)0;
	this -> g_err		= (int16_t)0;
	//Slew rate limiter
	this -> g_old_cmd	= (int16_t)0;
	this -> g_slew_rate = (int16_t)0;
	//Reset the PID unlock status: OK
	this -> g_b_unlock	= false;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return;	//OK
}	//end method: reset | void

/****************************************************************************
*****************************************************************************
**	PUBLIC STATIC METHODS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	PRIVATE METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Private Method
//!	init | void
/***************************************************************************/
//! @return no return
//!	@details
//! Initialize class vars
/***************************************************************************/

inline void Pid_s16::init( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Initialize PID parameters to invalid
	this -> g_cmd_max	= (int16_t)0;
	this -> g_cmd_min	= (int16_t)0;
	this -> g_sat_th	= (int16_t)0;

	//Initialize PID gains
	this -> g_kp		= (int16_t)0;
	this -> g_kd		= (int16_t)0;
	this -> g_ki		= (int16_t)0;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return;
}	//end method: init | void

/****************************************************************************
**	NAMESPACES
****************************************************************************/

} //End Namespace: Orangebot
