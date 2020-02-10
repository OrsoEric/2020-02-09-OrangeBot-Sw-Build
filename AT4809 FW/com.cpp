/****************************************************************
**	OrangeBot  Project
*****************************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************************
**
*****************************************************************************
**	Author: 			Orso Eric
**	Creation Date:
**	Last Edit Date:
**	Revision:			1
**	Version:			0.1 ALFA
****************************************************************************/

/****************************************************************************
**	HYSTORY VERSION
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	DESCRIPTION
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	KNOWN BUG
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	INCLUDE
****************************************************************************/

//type definition using the bit width and signedness
#include <stdint.h>
//General purpose macros
#include "at_utils.h"
//AT4809 PORT macros definitions
#include "at4809_port.h"
//from number to string
#include "string_uc.h"

#include "ctrl_pwm.h"
//Generic S16 PID Class
#include "pid_s16.h"

#include "global.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

//Slew rate limiter controller for the PWM channels
extern Orangebot::Ctrl_pwm g_vnh7040_pwm_ctrl;
//Each encoder has an associated PID controller
extern Orangebot::Pid_s16 g_vnh7040_pid[ NUM_ENC ];

/****************************************************************************
**	GLOBAL VARIABILE
****************************************************************************/
	
	///--------------------------------------------------------------------------
	///	PERFORMANCE
	///--------------------------------------------------------------------------
	
//Number of main loop execution per system tick
uint32_t g_board_performance;
//Number of bytes received from UART
uint32_t g_board_rpi_rxi_bandwidth;
//Number of bytes transmitted from UART
uint32_t g_board_rpi_txo_bandwidth;

/****************************************************************************
**	FUNCTION
****************************************************************************/

/***************************************************************************/
//!	@brief function
//!	error | Error_code
/***************************************************************************/
//! @param err_code | (Error_code) caller report the error code experienced by the system
//! @return void |
//! @details
//!	Error code handler function
/***************************************************************************/

void report_error( Error_code err_code )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	uint8_t t;
	//length of number string
	uint8_t ret_len;
	//numeric code
	uint8_t msg[User::String_uc::Size::STRING_U8];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//error code
	uint8_t u8_err_code = (uint8_t)err_code;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Construct numeric string
	ret_len = User::String_uc::to_string<uint8_t>( u8_err_code, msg );
	//Error command
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	//Send numeric string
	for (t = 0;t < ret_len;t++)
	{
		//Send number
		AT_BUF_PUSH( rpi_tx_buf, msg[t] );
	}
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End Function: error | Error_code

/***************************************************************************/
//!	function
//!	send_msg_ctrl_mode
/***************************************************************************/
//! @return void |
//! @brief Inform that control mode has been switched
//! @details
/***************************************************************************/

void send_msg_ctrl_mode( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	uint8_t t;
	//length of number string
	uint8_t ret_len;
	//numeric code
	uint8_t msg[User::String_uc::Size::STRING_U8];

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	AT_BUF_PUSH( rpi_tx_buf, 'C' );
	AT_BUF_PUSH( rpi_tx_buf, 'T' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	AT_BUF_PUSH( rpi_tx_buf, 'L' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'M' );
	AT_BUF_PUSH( rpi_tx_buf, 'O' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	
	//Construct numeric string
	ret_len = User::String_uc::to_string<uint8_t>( g_control_mode, msg );
	//Send numeric string
	for (t = 0;t < ret_len;t++)
	{
		//Send number
		AT_BUF_PUSH( rpi_tx_buf, msg[t] );
	}
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: send_msg_ctrl_mode | void

/***************************************************************************/
//!	function
//!	send_message_pwm_dual | void
/***************************************************************************/
//! @param x |
//! @return void |
//! @brief
//! @details
//!	Send current PWM setting
/***************************************************************************/

void send_message_pwm_dual( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//counter
	uint8_t t, ti;

	int16_t s16_tmp;

	uint8_t str_tmp[ User::String_uc::Size::STRING_S16 ];
	uint8_t str_len;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	
	AT_BUF_PUSH( rpi_tx_buf, 'P' );
	AT_BUF_PUSH( rpi_tx_buf, 'W' );
	AT_BUF_PUSH( rpi_tx_buf, 'M' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	AT_BUF_PUSH( rpi_tx_buf, 'U' );
	AT_BUF_PUSH( rpi_tx_buf, 'A' );
	AT_BUF_PUSH( rpi_tx_buf, 'L' );
	
	for (t = 0; t < 2;t++)
	{
		//Convert from PWM to number
		s16_tmp = g_vnh7040_pwm_ctrl.target(t);
		//Convert to string
		str_len = User::String_uc::to_string<int16_t>( s16_tmp, str_tmp );
		
		//Send string
		for (ti = 0;ti < str_len;ti++)
		{
			
			AT_BUF_PUSH( rpi_tx_buf, str_tmp[ti] );
			
		}
		//If not last argument
		if (t < 2-1)
		{
			//Add argument separator
			AT_BUF_PUSH( rpi_tx_buf, ':' );
		}
		
	}
	AT_BUF_PUSH( rpi_tx_buf, '\0' );
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: send_message_pwm_dual | void

/***************************************************************************/
//!	@brief function
//!	send_message_performance | void
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
//!	U32: main cycles per second
//! U8: KBytes RX in a second
//! U8: KBytes TX in a second
/***************************************************************************/

void send_message_performance( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Temp string sized for an uint32_t
	uint8_t str[ User::String_uc::Size::STRING_S32 ];
	//Length of a string
	uint8_t length;
	//Counter
	uint8_t t;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Send a encoder absolute position message
	AT_BUF_PUSH( rpi_tx_buf, 'S' );
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'T' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'P' );
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	AT_BUF_PUSH( rpi_tx_buf, 'F' );

		//----------------------------------------------------------------
		//	CPU LOAD
		//----------------------------------------------------------------
		//! Send U32 main loop executions per second
		
	//Construct string
	length = User::String_uc::to_string<int32_t>( g_board_performance, str );
	//Reset CPU load performance counter
	g_board_performance = 0;
	//For each string character
	for (t = 0;t < length;t++)
	{
		//Push the number of characters
		AT_BUF_PUSH( rpi_tx_buf, str[t] );
	}
	//Send argument separator
	AT_BUF_PUSH( rpi_tx_buf, ':' );

		//----------------------------------------------------------------
		//	UART BANDWIDTH
		//----------------------------------------------------------------
		// Send U8 RX bandwidth in KB/s

	//Fetch raw bandwidth B/s
	uint32_t tmp32 = g_board_rpi_rxi_bandwidth;
	//Reset UART load performance counter
	g_board_rpi_rxi_bandwidth = 0;
	//Convert to KB/s and clip to U8 limit
	tmp32 /= UART_BANDWIDTH_SCALE;
	tmp32 = AT_SAT( tmp32, 255, 0 );
	//Convert to U8
	uint8_t tmp8 = tmp32;
	//Construct string
	length = User::String_uc::to_string<uint8_t>( tmp8, str );
	//For each string character
	for (t = 0;t < length;t++)
	{
		//Push the number of characters
		AT_BUF_PUSH( rpi_tx_buf, str[t] );
	}

	//Send argument separator
	AT_BUF_PUSH( rpi_tx_buf, ':' );
	
	//Fetch raw bandwidth B/s
	tmp32 = g_board_rpi_txo_bandwidth;
	//Reset UART load performance counter
	g_board_rpi_txo_bandwidth = 0;
	//Convert to KB/s and clip to U8 limit
	tmp32 /= UART_BANDWIDTH_SCALE;
	tmp32 = AT_SAT( tmp32, 255, 0 );
	//Convert to U8
	tmp8 = tmp32;
	//Construct string
	length = User::String_uc::to_string<uint8_t>( tmp8, str );
	//For each string character
	for (t = 0;t < length;t++)
	{
		//Push the number of characters
		AT_BUF_PUSH( rpi_tx_buf, str[t] );
	}
	
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: send_message_performance | void

/***************************************************************************/
//!	@brief send_message_pid_err_dual | void
//!	send_message_performance | void
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
//! Send error and slew rate of two active PIDs to the master
/***************************************************************************/

void send_message_pid_err_dual( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Temp string sized for an uint32_t
	uint8_t str[ User::String_uc::Size::STRING_S16 ];
	//Length of a string
	uint8_t length;
	//Counters
	uint8_t t, ti;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Send a encoder absolute position message
	AT_BUF_PUSH( rpi_tx_buf, 'S' );
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'T' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'P' );
	AT_BUF_PUSH( rpi_tx_buf, 'I' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'E' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	AT_BUF_PUSH( rpi_tx_buf, 'R' );
	AT_BUF_PUSH( rpi_tx_buf, '_' );
	AT_BUF_PUSH( rpi_tx_buf, 'D' );
	AT_BUF_PUSH( rpi_tx_buf, 'U' );
	AT_BUF_PUSH( rpi_tx_buf, 'A' );
	AT_BUF_PUSH( rpi_tx_buf, 'L' );

	//----------------------------------------------------------------
	//	PID0 ERROR AND SLEW RATE
	//----------------------------------------------------------------
	//! Send PID performance statistics to the RPI
	
	//For each encoder and PID
	for (t = 0;t < NUM_ENC;t++)
	{
			//Send PID[t] Error
		//Construct string
		length = User::String_uc::to_string<int16_t>( g_vnh7040_pid[t].get_err(), str );
		//For each string character
		for (ti = 0;ti < length;ti++)
		{
			//Push the number of characters
			AT_BUF_PUSH( rpi_tx_buf, str[ti] );
		}
		//Send argument separator
		AT_BUF_PUSH( rpi_tx_buf, ':' );
		
			//Send PID[t] Slew Rate
		//Construct string
		length = User::String_uc::to_string<int16_t>( g_vnh7040_pid[t].get_slew_rate(), str );
		//For each string character
		for (ti = 0;ti < length;ti++)
		{
			//Push the number of characters
			AT_BUF_PUSH( rpi_tx_buf, str[ti] );
		}
		//If it's not the last argument
		if (t < (NUM_ENC -1))
		{
			//Send argument separator
			AT_BUF_PUSH( rpi_tx_buf, ':' );	
		}
	}	//End For: each encoder and PID
	
	//Send terminator
	AT_BUF_PUSH( rpi_tx_buf, '\0' );

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: send_message_pid_err_dual | void
