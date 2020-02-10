#ifndef GLOBAL_H
	//header environment variable, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define GLOBAL_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	#define F_CPU 20000000

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------

	#define RPI_RX_BUF_SIZE		64
	#define RPI_TX_BUF_SIZE		64

		///----------------------------------------------------------------------
		///	TIMINGS
		///----------------------------------------------------------------------
		//	Based on slow system tick.
		//	The TOP is a number that is counted. The number of counts is TOP+1
		
	typedef enum _Prescaler
	{
		//How many fast ticks make a control system tick
		TOP_CTRL_SYS = 0,
		//How many fast ticks make a slow tick
		TOP_SLOW_TICK = 39,
		//How many slow ticks without a valid message results in a communication timeout event
		TOP_COM_TIMEOUT = 99,
		//How many slow ticks are needed to toggle the green activity LED at slow and fast blink rate
		TOP_FAST_LED = 19,
		TOP_SLOW_LED = 49,
		//How often AT4809 sends its periodic messages
		TOP_PERIODIC_MSG = 49
	} Prescaler; 

		///----------------------------------------------------------------------
		///	COMMUNICATION
		///----------------------------------------------------------------------
	
	//Number and sequence of periodic messages sent by the board
	typedef enum _Periodic_msg
	{
		MSG_PERFORMANCE,
		MSG_PID_ERR,
		MSG_PWM,
		PERIODIC_MSG_NUM
	} Periodic_msg;
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
	
	//Maximum size of a signature string
	#define MAX_SIGNATURE_LENGTH	32
	//How much divide the UART bandwidth performance counter
	#define UART_BANDWIDTH_SCALE	1
	
		///----------------------------------------------------------------------
		///	VNH7040 DC MOTOR CONTROLLER
		///----------------------------------------------------------------------
	
	//Number of VNH7040 DC Motor drivers installed
	#define NUM_VNH7040				4
	//PWM limits
	#define MIN_VNH7040_PWM			16		//Too low PWM settings do not result in motion
	#define MAX_VNH7040_PWM			200		//Limit maximum PWM setting
	#define MAX_VNH7040_PWM_SLOPE	10		//Maximum PWM change in PWM unit per control system tick. In change of direction is forced at least one pass from zero
	//Set which motors need to be controlled in reverse to achieve forward motion with positive PWM
	#define LAYOUT_VNH7040_REVERSE	MASK(1)
	//Set which encoders need their reading applied in reverse to read positive going forward
	#define LAYOUT_ENC_REVERSE		MASK(1)

		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------
	
	//Number of quadrature encoders
	#define NUM_ENC				2
	//Threshold upon which local counters are synced with global counters
	#define ENC_UPDATE_TH		100
	//Number of times allowed to retry an update of global encoder vars before failing
	#define ENC_UPDATE_RETRY	3
	
		///----------------------------------------------------------------------
		///	PID
		///----------------------------------------------------------------------
		
	//Number of a PID gain parameters
	enum
	{
		PID_GAIN_KP			= 0,
		PID_GAIN_KI			= 1,
		PID_GAIN_KD			= 2,
		NUM_PID_GAINS		= 3
	};
	
	//If PID is saturated for this number of updates, the PID is unlocked in error
	#define PID_SAT_TH			250
	
	/****************************************************************************
	**	ENUM
	****************************************************************************/
		
	//Control modes
	typedef enum _Control_mode
	{
		CONTROL_STOP,	//Emergency stop mode
		CONTROL_PWM,		//PWM control mode. Just a slew rate limiter between user and drivers
		CONTROL_FAST_SPD,	//Speed control/Speed PID. When speed reading is above a given threshold
		CONTROL_SLOW_SPD,	//Speed control/Integral reference Position PID. When speed is below a given threshold
		CONTROL_POS			//Position control/Position PID
	} Control_mode;

	//Error codes that can be experienced by the program
	typedef enum _Error_code
	{
		OK,
		ERR_CODE_UNDEFINED_CONTROL_SYSTEM,
		ERR_CODE_COMMUNICATION_TIMEOUT,
		ERR_BAD_PARSER_DICTIONARY,
		ERR_UNIPARSER_RUNTIME,
		ERR_BAD_BOARD_SIGN,
		ERR_BAD_PARSER_RUNTIME_ARGUMENT,
		ERR_ENC_RETRY,							//The encoder failed to update global vars within its allowed retries
		ERR_PID_UNLOCK,							//A PID controller has become unlocked. This can happen due to command saturation or command slew rate limit.
		ERR_RX_BUF_OVERFLOW,					//An overflow event happened on the RX buffer
		ERR_UNKNOWN_PERIODIC_MGS				//No periodic message with this index exists
	} Error_code;

	/****************************************************************************
	**	MACRO
	****************************************************************************/

		///----------------------------------------------------------------------
		///	LEDS
		///----------------------------------------------------------------------

	#define LED0_TOGGLE()	\
		SET_BIT( PORTF.OUTTGL, 5 );

	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	//Global flags raised by ISR functions
	typedef struct _Isr_flags Isr_flags;

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	//Global flags raised by ISR functions
	struct _Isr_flags
	{
		//First byte
		uint8_t system_tick		: 1;	//System Tick
		uint8_t ctrl_updt		: 1;	//Control System
		uint8_t enc_sem			: 1;	//true = Encoder ISR is forbidden to write into the 32b encoder counters
		uint8_t enc_updt		: 1;	//true = Encoder ISR is forced to update the 32b counters and clear this flag if possible.
		uint8_t rx_buf_ovf		: 1;	//true = an overflow occurred on the RX buffer
		uint8_t slow_tick		: 1;	//Slow tick
		uint8_t					: 1;	//unused bits
	};

	/****************************************************************************
	**	PROTOTYPE: INITIALISATION
	****************************************************************************/

	//port configuration and call the peripherals initialization
	extern void init( void );

	/****************************************************************************
	**	PROTOTYPE: FUNCTION
	****************************************************************************/
		
		///----------------------------------------------------------------------
		///	COMMUNICATION
		///----------------------------------------------------------------------

	//Signal error
	void report_error( Error_code err_code );
	//Error code handler function
	void send_msg_ctrl_mode( void );
	//Send current PWM setting
	void send_message_pwm_dual( void );
	//Send board performance message to the RPI. Reset performance counters
	void send_message_performance( void );
	
	
		///----------------------------------------------------------------------
		///	PARSER
		///----------------------------------------------------------------------
		//	Handlers are meant to be called automatically when a command is decoded
		//	User should not call them directly
	
	//Handle Ping message coming from the RPI 3B+
	extern void ping_handler( void );
	//Handle Request for signature coming from the RPI 3B+
	extern void send_signature_handler( void );
	//Send board current performance
	extern void send_performance_handler( uint8_t enable );
	//Robot independent messages
	extern void set_platform_pwm_handler( int16_t right, int16_t left );
	extern void set_platform_spd_handler( int16_t right, int16_t left );
	extern void set_platform_pos_handler( int32_t right, int32_t left );
	//Handle request for absolute encoder position
	extern void send_enc_pos_handler( uint8_t index );
	//Handle request for all encoder speed
	extern void send_enc_spd_handler( void );
	//Master asks for current speed PID settings. Fourth parameter is fixed point position
	extern void get_spd_pid_handler( void );
	//RPI wants to set the speed PID gain
	extern void set_spd_pid_handler( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd );
	//RPI wants to set the position PID gain
	extern void set_pos_pid_handler( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd );
	//Master asks for current error and slew rate of two active PID controllers
	extern void get_pid_err_dual_handler( void );
	//Send error and slew rate of two active PIDs to the master
	extern void send_message_pid_err_dual( void );

		///----------------------------------------------------------------------
		///	VNH7040 MOTORS
		///----------------------------------------------------------------------
		//	HAL API to control the VNH7040 drivers
		
	//Set direction and pwm setting of a VNH7040 controlled motor
	extern bool set_vnh7040_pwm( uint8_t index, int16_t pwm );

		///----------------------------------------------------------------------
		///	PLATFORM HAL API
		///----------------------------------------------------------------------
		//	HAL API. Abstract motor layout
	
	//Control PWM of the platform to move according to the layout
	extern bool set_platform_pwm( int16_t right, int16_t left );
	//Control SPD of the platform to move according to the layout
	extern bool set_platform_spd( int16_t right, int16_t left );
	//Set target for the Position Control System taking into account the VNH7040 layout
	extern bool set_platform_pos( int32_t right, int32_t left );
	
		///----------------------------------------------------------------------
		///	ENCODERS
		///----------------------------------------------------------------------

	//Decode four quadrature encoder channels
	extern void quad_encoder_decoder( uint8_t enc_in );
	//Force an update and save the 32b encoder counters in an input vector
	extern bool get_enc_cnt( int32_t *enc_cnt );
	//Force update of the global 32b counters and compute position and speed
	extern bool process_enc( void );
	
		///----------------------------------------------------------------------
		///	CONTROL SYSTEM
		///----------------------------------------------------------------------

	//Initialize the control systems
	extern bool init_ctrl_system( void );
	//Handle switch between control systems and execute the correct control system. Handles the stop mode.
	extern bool control_system( void );
	//Set the Speed PID target gain for all encoder channels and force control system to reload
	extern bool set_spd_pid( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd );
	//Set the Position PID target gain for all encoder channels and force control system to reload
	extern bool set_pos_pid( int16_t gain_kp, int16_t gain_ki, int16_t gain_kd );
	//Called if a PID is unable to keep a lock
	extern void pid_unlock_handler( void );
	
	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	STATUS FLAGS
		///----------------------------------------------------------------------

	//Volatile flags used by ISRs
	extern volatile	Isr_flags g_isr_flags;

		///--------------------------------------------------------------------------
		///	PERFORMANCE
		///--------------------------------------------------------------------------

	//Enable board performance profiling
	extern bool g_b_enable_performance;
	//Number of main loop execution per system tick
	extern uint32_t g_board_performance;
	//Number of bytes received from UART
	extern uint32_t g_board_rpi_rxi_bandwidth;
	//Number of bytes transmitted from UART
	extern uint32_t g_board_rpi_txo_bandwidth;
	
		///----------------------------------------------------------------------
		///	BUFFERS
		///----------------------------------------------------------------------
		//	Buffers structure and data vectors

	//Safe circular buffer for UART input data
	extern volatile At_buf8_safe rpi_rx_buf;
	//Safe circular buffer for uart tx data
	extern At_buf8 rpi_tx_buf;
	//allocate the working vector for the buffer
	extern uint8_t v0[ RPI_RX_BUF_SIZE ];
	//allocate the working vector for the buffer
	extern uint8_t v1[ RPI_TX_BUF_SIZE ];
	
		///--------------------------------------------------------------------------
		///	PARSER
		///--------------------------------------------------------------------------

	//Board Signature
	extern uint8_t *g_board_sign;
	//communication timeout counter
	extern uint8_t g_uart_timeout_cnt;
	//Communication timeout has been detected
	extern bool g_f_timeout_detected;
	
		///--------------------------------------------------------------------------
		///	VNH7040 DC MOTOR CONTROLLER
		///--------------------------------------------------------------------------
	
	
	
		///--------------------------------------------------------------------------
		///	ENCODERS
		///--------------------------------------------------------------------------
	
	//Global 32b encoder counters
	extern volatile int32_t g_enc_cnt[NUM_ENC];
	//Encoder absolute position
	extern int32_t g_enc_pos[NUM_ENC];
	//Encoder speed
	extern int16_t g_enc_spd[NUM_ENC];
		
		///--------------------------------------------------------------------------
		///	CONTROL SYSTEM
		///--------------------------------------------------------------------------
	
	//Actual and target control system mode
	extern Control_mode g_control_mode;
	extern Control_mode g_control_mode_target;
	//Store gains for the Speed PIDs and the Position PIDs
	extern int16_t g_pid_spd_gain[NUM_PID_GAINS];
	extern int16_t g_pid_pos_gain[NUM_PID_GAINS];
	//Control System References
	extern int16_t g_pwm_target[NUM_VNH7040];
	extern int16_t g_pid_spd_target[NUM_ENC];	
	extern int32_t g_pid_pos_target[NUM_ENC];
	
#else
	#warning "multiple inclusion of the header file global.h"
#endif


