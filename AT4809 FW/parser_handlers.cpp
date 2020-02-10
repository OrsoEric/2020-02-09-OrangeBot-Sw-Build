
/****************************************************************
**	INCLUDES
****************************************************************/

//type definition using the bit width and signedness
#include <stdint.h>

//General purpose macros
#include "at_utils.h"

#include "string_uc.h"

#include "global.h"
//Universal Parser V5
#include "uniparser.h"
//PID Control system
#include "pid_s16.h"

/****************************************************************
** GLOBAL VARS PROTOTYPES
****************************************************************/

//Control system
extern Orangebot::Pid_s16 g_vnh7040_pid[ NUM_ENC ];

/****************************************************************
** GLOBAL VARS
****************************************************************/

	///--------------------------------------------------------------------------
	///	PERFORMANCE
	///--------------------------------------------------------------------------

//Enable board performance profiling
bool g_b_enable_performance;

	///--------------------------------------------------------------------------
	///	PARSER
	///--------------------------------------------------------------------------

//Board Signature
uint8_t *g_board_sign = (uint8_t *)"OrangeBot-2020-02-06";
//communication timeout counter
uint8_t g_uart_timeout_cnt = 0;
//Communication timeout has been detected
bool g_f_timeout_detected = false;

/***************************************************************************/
//!	function
//!	init_parser_commands | Orangebot::Uniparser &
/***************************************************************************/
//! @param parser_tmp | Orangebot::Uniparser | Parser to which commands are to be atached
//! @return bool | false = OK | true = command was invalid and was not registered
//! @brief
//! @details
/***************************************************************************/

bool init_parser_commands( Orangebot::Uniparser &parser_tmp )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Return flag
	bool f_ret = false;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//! Register commands and handler for the universal parser class. A masterpiece :')
	//Register ping command. It's used to reset the communication timeout
	f_ret = parser_tmp.add_cmd( "P", (void *)&ping_handler );
	//Register the Find command. Board answers with board signature
	f_ret |= parser_tmp.add_cmd( "F", (void *)&send_signature_handler );
	
	//Add command to get current board performance
	f_ret |= parser_tmp.add_cmd( "GET_PERFORMANCE%u", (void *)&send_performance_handler );
	
	//Platform set PWM command
	f_ret |= parser_tmp.add_cmd( "ROBOT_PWM%S:%S", (void *)&set_platform_pwm_handler );
	//Platform set SPD command
	f_ret |= parser_tmp.add_cmd( "ROBOT_SPD%S:%S", (void *)&set_platform_spd_handler );
	//Platform set POS command
	f_ret |= parser_tmp.add_cmd( "ROBOT_POS%d:%d", (void *)&set_platform_pos_handler );
	
	//Master asks for absolute encoder position
	f_ret |= parser_tmp.add_cmd( "ENC_ABS%u", (void *)&send_enc_pos_handler );
	//Master asks for encoder speed
	f_ret |= parser_tmp.add_cmd( "ENC_SPD", (void *)&send_enc_spd_handler );
	
	//Master asks for current speed PID settings. Fourth parameter is fixed point position
	f_ret |= parser_tmp.add_cmd( "GET_SPD_PID", (void *)&get_spd_pid_handler );
	//Master asks for current speed PID settings
	f_ret |= parser_tmp.add_cmd( "SET_SPD_PID%S:%S:%S", (void *)&set_spd_pid_handler );
	
	//Master asks for current speed PID settings. Fourth parameter is fixed point position
	f_ret |= parser_tmp.add_cmd( "GET_POS_PID", (void *)&get_spd_pid_handler );
	//Master asks for current speed PID settings
	f_ret |= parser_tmp.add_cmd( "SET_POS_PID%S:%S:%S", (void *)&set_pos_pid_handler );
	//Master asks for error of PID
	f_ret |= parser_tmp.add_cmd( "GET_PID_ERR_DUAL", (void *)&get_pid_err_dual_handler );
	
	//If: Uniparser V4 failed to register a command
	if (f_ret == true)
	{
		//Signal a likely syntax error
		report_error( ERR_BAD_PARSER_DICTIONARY );
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return f_ret;
}	//End function: init_parser_commands | Orangebot::Uniparser &

/***************************************************************************/
//!	@brief ping command handler
//!	ping_handler | void
/***************************************************************************/
//! @return void
//!	@details
//! Handler for the ping command. Keep alive connection
/***************************************************************************/

void ping_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: ping_handler | void

/***************************************************************************/
//!	@brief board signature handler
//!	signature_handler | void
/***************************************************************************/
//! @return void
//!	@details
//! Handler for the get board signature command. Send board signature via UART
/***************************************************************************/

void send_signature_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;
	//Temp return
	uint8_t ret;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;
	//Length of the board signature string
	uint8_t board_sign_length = User::String_uc::str_length( g_board_sign, MAX_SIGNATURE_LENGTH );
	//If: string is invalid
	if (board_sign_length > MAX_SIGNATURE_LENGTH)
	{
		report_error( Error_code::ERR_BAD_BOARD_SIGN );
		return; //FAIL
	}
	//Construct string length
	uint8_t str[User::String_uc::Size::STRING_U8];
	//Convert the string length to number
	ret = User::String_uc::to_string<uint8_t>( board_sign_length, str );
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Send a signature message with string argument to the RPI 3B+
	AT_BUF_PUSH( rpi_tx_buf, 'F' );
	//For: number of number of signature characters
	for (t = 0;t < ret;t++)
	{
		//Push the number of characters	
		AT_BUF_PUSH( rpi_tx_buf, str[t] );
	}
	//Send message terminator
	AT_BUF_PUSH( rpi_tx_buf, (uint8_t)0x00 );
	//For: number of signature characters
	for (t = 0;t < board_sign_length;t++)
	{
		//Send the next signature byte
		AT_BUF_PUSH(rpi_tx_buf, g_board_sign[t]);
	}
	//Send string terminator
	AT_BUF_PUSH( rpi_tx_buf, (uint8_t)0x00 );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: signature_handler | void

/***************************************************************************/
//!	@brief function
//!	send_performance_handler | uint8_t
/***************************************************************************/
//! @return void |
//! @details
//! Record the preference of the RPI about sending performance messages
/***************************************************************************/

void send_performance_handler( uint8_t enable )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	
	//Record the preference of the RPI about sending performance messages
	g_b_enable_performance = enable;
	/*
	AT_BUF_PUSH( rpi_tx_buf, 'O' );
	AT_BUF_PUSH( rpi_tx_buf, 'K' );
	AT_BUF_PUSH( rpi_tx_buf, '0'+ g_b_enable_performance);
	AT_BUF_PUSH( rpi_tx_buf, '\0' );
	*/
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: send_performance_handler | uint8_t

/***************************************************************************/
//!	function
//!	set_platform_pwm_handler | int16_t, int16_t
/***************************************************************************/
//! @param right | target to right side wheels
//! @param left | target to left side wheels
//! @return void |
//! @brief
//! @details
/***************************************************************************/

void set_platform_pwm_handler( int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//HAL API to control VNH7040 motors according to the platform layout
	set_platform_pwm( right, left );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: set_platform_pwm_handler | int16_t, int16_t

/***************************************************************************/
//!	@brief function
//!	set_platform_spd_handler | int16_t, int16_t
/***************************************************************************/
//! @param right | target to right side wheels
//! @param left | target to left side wheels
//! @return void |
//! @details
//! Set target speed to wheels on one side or the other
/***************************************************************************/

void set_platform_spd_handler( int16_t right, int16_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//HAL API to control VNH7040 motors according to the platform layout
	set_platform_spd( right, left );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: set_platform_spd_handler | int16_t, int16_t

/***************************************************************************/
//!	@brief function
//!	set_platform_pos_handler | int32_t, int32_t
/***************************************************************************/
//! @param right | target to right side wheels
//! @param left | target to left side wheels
//! @return void |
//! @details
//! Set target position to wheels on one side or the other
/***************************************************************************/

void set_platform_pos_handler( int32_t right, int32_t left )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//HAL API to control VNH7040 motors according to the platform layout
	set_platform_pos( right, left );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: set_platform_pos_handler | int32_t, int32_t

/***************************************************************************/
//!	@brief board signature handler
//!	send_enc_pos_handler | uint8_t
/***************************************************************************/
//! @param index | index of the encoder position to be sent
//! @return void
//!	@details
//! Handle request for absolute encoder position
/***************************************************************************/

void send_enc_pos_handler( uint8_t index )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;
	//return
	uint8_t ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//If: invalid encoder channel
	if (index >= NUM_ENC)
	{
		report_error(Error_code::ERR_BAD_PARSER_RUNTIME_ARGUMENT);
		return;	//FAIL
	}
	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Temp string sized for an int32_t
	uint8_t str[User::String_uc::Size::STRING_S32];
	//Construct index string
	ret = User::String_uc::to_string<uint8_t>( index, str );
	//Send a encoder absolute position message
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'N' );
	AT_BUF_PUSH( rpi_tx_buf, 'C' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'A' );
	AT_BUF_PUSH( rpi_tx_buf, 'B' );
	AT_BUF_PUSH( rpi_tx_buf, 'S' );
	//For each string character
	for (t = 0;t < ret;t++)
	{
		//Push the number of characters
		AT_BUF_PUSH( rpi_tx_buf, str[t] );
	}
	//Send argument separator
	AT_BUF_PUSH( rpi_tx_buf, ':' );	
	//Construct encoder position string
	ret = User::String_uc::to_string<int32_t>( g_enc_pos[ index ], str );
	//For each string character
	for (t = 0;t < ret;t++)
	{
		//Push the number of characters
		AT_BUF_PUSH( rpi_tx_buf, str[t] );
	}
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: send_enc_pos_handler | uint8_t


/***************************************************************************/
//!	@brief handler
//!	send_enc_spd_handler | void
/***************************************************************************/
//! @param index | index of the encoder position to be sent
//! @return void
//!	@details
//! Handle request for all encoder speed
/***************************************************************************/

void send_enc_spd_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t, ti;
	//return
	uint8_t ret;
	//Temp string sized for an int16_t
	uint8_t str[User::String_uc::Size::STRING_S16];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Send a encoder absolute position message
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'N' );
	AT_BUF_PUSH( rpi_tx_buf, 'C' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'S' );
	AT_BUF_PUSH( rpi_tx_buf, 'P' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	AT_BUF_PUSH( rpi_tx_buf, 'U' );
	AT_BUF_PUSH( rpi_tx_buf, 'A' );
	AT_BUF_PUSH( rpi_tx_buf, 'L' );
	
	//For: each encoder channel
	for (t = 0;t < NUM_ENC;t++)
	{
		//Construct encoder position string
		ret = User::String_uc::to_string<int16_t>( g_enc_spd[ t ], str );
		//For each string character
		for (ti = 0;ti < ret;ti++)
		{
			//Push the number of characters
			AT_BUF_PUSH( rpi_tx_buf, str[ti] );
		}
		//If not last argument
		if (t < NUM_ENC -1)
		{
			//Send argument separator
			AT_BUF_PUSH( rpi_tx_buf, ':' );
		}
		
	}
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return; //OK
}	//end handler: send_enc_spd_handler | void

/***************************************************************************/
//!	@brief function
//!	get_spd_pid_handler | void
/***************************************************************************/
//! @return void |
//! @details
//! Master asks for current speed PID settings. Fourth parameter is fixed point position
/***************************************************************************/

void get_spd_pid_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t, ti;
	//length of string
	uint8_t length;
	//String to hold PID parameters
	uint8_t str[User::String_uc::Size::STRING_S16];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Send answer
	AT_BUF_PUSH( rpi_tx_buf, 'S' );
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'T' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'S' );
	AT_BUF_PUSH( rpi_tx_buf, 'P' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'P' );
	AT_BUF_PUSH( rpi_tx_buf, 'I' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	
	int16_t tmp;
	
	//For: each gain parameter of a PID
	for (t = 0;t < NUM_PID_GAINS;t++)
	{
		//Switch: correct gain parameter
		switch (t)
		{
			case 0:
			{
				tmp = g_vnh7040_pid[0].gain_kp();
				break;
			}
			case 1:
			{
				tmp = g_vnh7040_pid[0].gain_ki();
				break;
			}
			case 2:
			{
				tmp = g_vnh7040_pid[0].gain_kd();
				break;
			}
			default:
			{
				tmp = 0;
				return; //FAIL
			}
		}
		//construct number
		length = User::String_uc::to_string<int16_t>( tmp, str );
		//For each number character
		for (ti = 0;ti < length;ti++)
		{
			//Send number
			AT_BUF_PUSH( rpi_tx_buf, str[ti] );
		}
		//If not the last parameter send the argument separator
		if (t < NUM_PID_GAINS -1)
		{
			//Push argument separator
			AT_BUF_PUSH( rpi_tx_buf, ':' );
		}
	} //End For: each gain parameter of a PID
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: get_spd_pid_handler | void

/***************************************************************************/
//!	@brief function
//!	set_spd_pid_handler | int16_t, int16_t, int16_t
/***************************************************************************/
//! @return void |
//! @details
//! Master asks for current speed PID settings. Fourth parameter is fixed point position
/***************************************************************************/

void set_spd_pid_handler( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	
	//Set all speed PID gains
	set_spd_pid( gain_kp, gain_ki, gain_kd );
	//Answer with the updated Parameters
	get_spd_pid_handler();
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: set_spd_pid_handler | int16_t, int16_t, int16_t

/***************************************************************************/
//!	@brief function
//!	set_pos_pid_handler | int16_t, int16_t, int16_t
/***************************************************************************/
//! @return void |
//! @details
//! Master asks for current speed PID settings. Fourth parameter is fixed point position
/***************************************************************************/

void set_pos_pid_handler( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	
	//Set all speed PID gains
	set_pos_pid( gain_kp, gain_ki, gain_kd );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: set_spd_pid_handler | int16_t, int16_t, int16_t



/***************************************************************************/
//!	@brief function
//!	get_pid_err_dual_handler | void
/***************************************************************************/
//! @return void |
//! @details
//! Master asks for current error and slew rate of two active PID controllers
/***************************************************************************/

void get_pid_err_dual_handler( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Reset communication timeout handler
	g_uart_timeout_cnt = 0;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	
	send_message_pid_err_dual();
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: get_pid_err_dual_handler | void