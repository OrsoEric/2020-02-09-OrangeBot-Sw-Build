/**********************************************************************************
**	ENVIROMENT VARIABILE
**********************************************************************************/

#ifndef PID_S16_H_
	#define PID_S16_H_

/**********************************************************************************
**	GLOBAL INCLUDES
**********************************************************************************/

/**********************************************************************************
**	DEFINES
**********************************************************************************/

/**********************************************************************************
**	MACROS
**********************************************************************************/

/**********************************************************************************
**	NAMESPACE
**********************************************************************************/

//! @namespace OrangeBot
namespace Orangebot
{

/**********************************************************************************
**	TYPEDEFS
**********************************************************************************/

typedef enum _Pid_s16_config
{
	//Limits of S16. Limits are symmetrical for this class
	MAX_S16					= INT16_MAX,
	MIN_S16					= -32767,
	//Floating point position of the PID gain parameters
	GAIN_FP					= 15,
	//Depth of the error memory for the derivative calculation
	ERR_MEM_SIZE			= 2,
	//Parameters for the deep asymmetric derivative computation
	DERIVATIVE_PARAM2		= +1,
	DERIVATIVE_PARAM1		= -3,
	DERIVATIVE_PARAM0		= +2,
	DERIVATIVE_PARAM_FP		= 0,
	//threshold expressed in (MAX/MIN)/2^n upon which the saturation threshold counter is increased
	SLEW_RATE_LIMIT			= 3,
	//Parameters for the Error low pass filter
	LP_ERR_GAIN				= 7,	//Gain for the low pass error estimation. Valid values are from 1 to (1<<PID_ERR_FP).
	LP_ERR_FP				= 3,	//Fixed point position for the low pass error estimation (1<<PID_ERR_FP)
	//Parameters for the Slew Rate low pass filter
	LP_SLEW_RATE_GAIN		= 7,
	LP_SLEW_RATE_FP			= 3,
	//Parameters for the two slope error gain
	TWO_SLOPE_INPUT_TH		= 16,
	TWO_SLOPE_OUTPUT_TH		= 128,
	TWO_SLOPE_GAIN_FP		= 0
} Pid_s16_config;

/**********************************************************************************
**	PROTOTYPE: STRUCTURES
**********************************************************************************/

/**********************************************************************************
**	PROTOTYPE: GLOBAL VARIABILES
**********************************************************************************/

/**********************************************************************************
**	PROTOTYPE: CLASS
**********************************************************************************/

/************************************************************************************/
//! @class 		Dummy
/************************************************************************************/
//!	@author		Orso Eric
//! @version	2020-01-28
//! @date		2019-11-08
//! @brief		PID
//! @details
//!	Advanced PID Library \n
//! FEATURES:	\n
//!		Command Saturation	\n
//! Detect when command is saturated \n
//! This inibiths the integrator from growing, improving response time \n
//! This allow a detection of PID unlock when the command is saturated for too long, meaning the PID can't keep up \n
//!		Error report	\n
//!
//! @pre		No prerequisites
//! @bug		None
//! @warning	No warnings
//! @copyright	License ?
//! @todo		todo list
/************************************************************************************/

class Pid_s16
{
	//Visible to all
	public:
		//--------------------------------------------------------------------------
		//	CONSTRUCTORS
		//--------------------------------------------------------------------------

		//! Default constructor
		Pid_s16( void );

		//--------------------------------------------------------------------------
		//	DESTRUCTORS
		//--------------------------------------------------------------------------

		//!Default destructor
		~Pid_s16( void );

		//--------------------------------------------------------------------------
		//	OPERATORS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	SETTERS
		//--------------------------------------------------------------------------

		//Set command limits
		bool set_limit_cmd( int16_t min, int16_t max );

		//--------------------------------------------------------------------------
		//	GETTERS
		//--------------------------------------------------------------------------

		//PID Status. false = OK | true = protection
		bool get_pid_status( void );
		//Error committed by the PID
		int16_t get_err( void );
		//Slew rate of the generated command. Give an indication of how hard a time the PID is having keeping a lock
		int16_t get_slew_rate( void );

		//--------------------------------------------------------------------------
		//	REFERENCES
		//--------------------------------------------------------------------------

		//PID limits
		int16_t &limit_cmd_max( void );
		int16_t &limit_cmd_min( void );
		uint16_t &limit_sat_th( void );

		//Set PID gain parameters
		int16_t &gain_kp( void );
		int16_t &gain_kd( void );
		int16_t &gain_ki( void );

		//--------------------------------------------------------------------------
		//	TESTERS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PUBLIC METHODS
		//--------------------------------------------------------------------------

		//Execute a step of the PID controller. RET_ERR mean that the PID is now unlocked
		int16_t exe( int16_t reference, int16_t feedback );
		//Overload to allow for 32b inputs and 32b feedback signals. Error must still be 16b for the PID to operate as intended. EG. this allows to subtract the absolute position offset.
		int16_t exe( int32_t reference, int32_t feedback );
		//Reset the PID
		void reset( void );

		//--------------------------------------------------------------------------
		//	PUBLIC STATIC METHODS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PUBLIC VARS
		//--------------------------------------------------------------------------

	//Visible to derived classes
	protected:
		//--------------------------------------------------------------------------
		//	PROTECTED METHODS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PROTECTED VARS
		//--------------------------------------------------------------------------

	//Visible only inside the class
	private:
		//--------------------------------------------------------------------------
		//	PRIVATE METHODS
		//--------------------------------------------------------------------------

		//Initialize class vars
		void init( void );

		//--------------------------------------------------------------------------
		//	PRIVATE VARS
		//--------------------------------------------------------------------------

		//PID Unlock. true: command has been pegged to 100% for too long and PID is disabled.
		bool g_b_unlock;

			//!	Parameters
		//PID core gain parameters
		int16_t g_kp, g_kd, g_ki;
		//Command Saturation limits. Maximum command allowed
		int16_t g_cmd_max, g_cmd_min;
		//If command is saturated a number of cycle bigger than this number, a PID unlock error is issued. Zero means that the detection is inactive
		uint16_t g_sat_th;

			//!	Status Variables
		//PID integral accumulator
		int16_t g_acc;
		//PID derivative memory buffer
		int16_t g_old_err[ Pid_s16_config::ERR_MEM_SIZE ];
		//Previous command memory. used to compute command slew rate
		int16_t g_old_cmd;
		//Counter that stores the number of consecutive execution in which command is saturated
		uint16_t g_sat_cnt;
		//PID Error
		int16_t g_err;
		//Slew rate of the command. The same LP filter to error applies
		int16_t g_slew_rate;
};	//End Class: Pid_s16

/**********************************************************************************
**	NAMESPACE
**********************************************************************************/

} //End Namespace

#else
    #warning "Multiple inclusion of hader file"
#endif
