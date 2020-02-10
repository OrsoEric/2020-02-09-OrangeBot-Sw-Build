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
**	GLOBAL VARS
****************************************************************************/

//Global 32b encoder counters. Shared by EncoderISR.
volatile int32_t g_enc_cnt[NUM_ENC];
//Encoder absolute position
int32_t g_enc_pos[NUM_ENC];
//Encoder speed
int16_t g_enc_spd[NUM_ENC];

//----------------------------------------------------------------
// Encoder LUT
//----------------------------------------------------------------
// Bit 765 | unused, hold at zero
// Bit 4 | Previous direction | 0 = clockwise | 1 = counterclockwise
// Bit 32 | old encoder reading | B channel A channel
// Bit 10 | new encoder reading | B channel A channel

const int8_t enc_lut[32] =
{
	(int8_t)+0,	//No Change
	(int8_t)-1,	//B Rise with A=0: -1 (Counter Clockwise)
	(int8_t)+1,	//A Rise with B=0: +1 (Clockwise)
	(int8_t)+2,	//A Rise B Rise: double event +2
	(int8_t)+1,	//B Fall with A=0: +1 (Clockwise)
	(int8_t)+0,	//No Change
	(int8_t)+2,	//A Rise B Fall: double event +2
	(int8_t)-1,	//A Rise with B=1: -1 (Counter Clockwise)
	(int8_t)-1,	//A Fall with B=0: -1 (Counter Clockwise)
	(int8_t)+2,	//A Fall B Rise: double event +2
	(int8_t)+0,	//No Change
	(int8_t)+1,	//B Rise with A=1: +1 (Clockwise)
	(int8_t)+2,	//A Fall B Fall: double event +2
	(int8_t)+1,	//A Fall with B=1: +1 (Clockwise)
	(int8_t)-1,	//B Fall with A=1: -1 (Counter Clockwise)
	(int8_t)+0,	//No Change
	(int8_t)+0,	//No Change
	(int8_t)-1,	//B Rise with A=0: -1 (Counter Clockwise)
	(int8_t)+1,	//A Rise with B=0: +1 (Clockwise)
	(int8_t)-2,	//A Rise B Rise: double event -2
	(int8_t)+1,	//B Fall with A=0: +1 (Clockwise)
	(int8_t)+0,	//No Change
	(int8_t)-2,	//A Rise B Fall: double event -2
	(int8_t)-1,	//A Rise with B=1: -1 (Counter Clockwise)
	(int8_t)-1,	//A Fall with B=0: -1 (Counter Clockwise)
	(int8_t)-2,	//A Fall B Rise: double event -2
	(int8_t)+0,	//No Change
	(int8_t)+1,	//B Rise with A=1: +1 (Clockwise)
	(int8_t)-2,	//A Fall B Fall: double event -2
	(int8_t)+1,	//A Fall with B=1: +1 (Clockwise)
	(int8_t)-1,	//B Fall with A=1: -1 (Counter Clockwise)
	(int8_t)+0	//No Change
};

/****************************************************************************
**	FUNCTION
****************************************************************************/

/****************************************************************************
**  Function
**  quad_encoder_decoder
****************************************************************************/
//! @brief Decode four quadrature encoder channels on an edge on each of the channel
//! @details
//!	PIN	| ENC0	| ENC1	| ENC2	| ENC3	|
//!	-------------------------------------
//!	CHA	| PC0	| PC2	| PC4	| PC6	|
//!	CHB	| PC1	| PC3	| PC5	| PC7	|
//!
//!	FEATURES:
//!
//!		combined ISR
//! option 1 was to write four smaller ISRs one for channel
//! option 2 was to write one bigger ISR triggered by each edge
//!	which one is better depends on ISR overhead and load distribution between channels
//! since all encoders go at the same speed, it make sense to write just one routine?
//!
//! 	global sync
//!	ISR only updates a local smaller faster counter all of the times.
//! synchronization between this counter and the main routine only happens when
//! the main routine is not sampling it, when the main routine is requesting it or when local counters are getting too full
//! this feature is meant to reduce the overhead of the ISR encoder routine significantly by not updating 32bit registers
//!
//!     double event
//!	A LUT allows handling of tricky double events that happen when the ISR can't keep up and skip a beat
//! double event can be handled with no error in count and allow to warn the main that the encoders are getting out of hand
//! and stalling the micro controller
//!
//! ALGORITHM:
//! >Fetch new pin configuration
//! >For each channel
//!		>Build an index to the encoder LUT
//!		>Decode the increment to be added to the 16b relative counter
//!		>Save direction and detect double events to raise warnings
//!	>Write new configuration into old configuration
/***************************************************************************/

void quad_encoder_decoder( uint8_t enc_in )
{
	//----------------------------------------------------------------
	//	STATICS
	//----------------------------------------------------------------

	//relative encoder counters
	static int8_t enc_cnt[NUM_ENC] = { 0 };
	//Memory of the previous direction of the encoders. each bit is one encoder channel. false=+ true=-
	static uint8_t enc_dir = (uint8_t)0x00;
	//Memory of previous encoder pin configuration. Initialize to current one at first cycle
	static uint8_t enc_pin_old = enc_in;
	
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------
	
	//Fetch pin configuration
	uint8_t enc_pin = enc_in;
	//Counter used to scan the encoders
	uint8_t t;
	//index to the LUT
	uint8_t index;
	//increment decoded from the LUT
	int8_t increment;
	//temporary error counter
	bool f_err = false;
	//this flag is used to detect when a preventive overflow update is required
	bool f_update = false;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//For: each encoder channel
	for (t = 0;t < NUM_ENC;t++)
	{
		//! Build address to the encoder LUT
		// | 4		| 3		| 2		| 1		| 0
		// | dir	| old B	| old A	| B		| A
		//Inject new AB and clear index
		index = (((enc_pin) >> (2*t)) & 0x03);
		//Inject old AB
		index |= ((t==0)?((enc_pin_old<<2)&0x0c):(((enc_pin_old) >> (2*(t-1))) & 0x0c));
		//Inject old direction
		index |= (((enc_dir) << (4 -t)) & 0x10);

		//! Decode the increment through the LUT
		//Use the index as address for the encoder LUT, applying the complex truth table
		increment = enc_lut[ index ];
		//! Layout correction
		if (IS_BIT_ONE(LAYOUT_ENC_REVERSE, t))
		{
			//Apply the decoded increment in reverse
			increment = -increment;
		}
		//! Apply increment to local relative memory and compute special
		//Apply increment
		enc_cnt[t] += increment;
		//Compute new direction flag and write it back to the correct bit of the encoder direction memory
		enc_dir = (enc_dir & INV_MASK(t)) | (((increment < 0) & 0x01) << t);
		//Detect if a double event happened and remember it. Serves as over speed warning
		f_err |= ((increment == +2) || (increment == -2));
		//overflow update flag. if at least a counter is getting dangerously large
		f_update |= ((enc_cnt[t] >= ENC_UPDATE_TH) || (enc_cnt[t] <= -ENC_UPDATE_TH));

	} //End For: each encoder channel

	//! Write back double event error flag
	//g_isr_flags.enc_double_event |= f_err;

	//! Write back ISR counters to global 32bit counters
	//Only write back if main requests it, if at least one counter is above threshold. withhold if someone is accessing the global counter riht now
	if ((g_isr_flags.enc_sem == false) && ((f_update == true) || (g_isr_flags.enc_updt == true)))
	{
		//notify the main that sync happened
		g_isr_flags.enc_updt = false;
		//For: each encoder channel
		for (t = 0;t < NUM_ENC;t++)
		{
			//Synchronize with the global 32b counters
			g_enc_cnt[t] += enc_cnt[t];
			//Clear the local 8b counters
			enc_cnt[t] = 0;
		} //End For: each encoder channel
	}

	//Save pin configuration
	enc_pin_old = enc_pin;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
} //End function: quad_encoder_decoder

/***************************************************************************/
//!	function
//!	get_enc_cnt
/***************************************************************************/
//! @param enc_cnt | int32_t vector. Function returns in this vector the value of the global encoder counters
//! @return bool | false=OK | true=failed to update global counters
//! @brief Force an update and save the 32b encoder counters in an input vector
//! @details
//!		Algorithm:
//!	>disable interrupt
//!	>raise sync flag
//!	>manually execute encoder decoding routine
//! >Transfer register value to local vars
/***************************************************************************/

bool get_enc_cnt( int32_t *enc_cnt )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Counter
	uint8_t t;
	//return flag
	bool f_ret;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Disable interrupts
	cli();
	//Force update of 32b global counters
	g_isr_flags.enc_updt = true;

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Decode encoders and update 32b counters
	quad_encoder_decoder( PORTC.IN );
	//Enable interrupts
	sei();
	//If encoder routine failed to update global counters
	f_ret = g_isr_flags.enc_updt;
	//For: all encoder channels
	for (t = 0;t < NUM_ENC;t++)
	{
		//Copy
		enc_cnt[t] = g_enc_cnt[t];
	}
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	//false = OK | true = fail (global counters failed to update)
	return f_ret;
}

/***************************************************************************/
//!	@brief function
//!	process_enc | void
/***************************************************************************/
//! @return bool | true = global counters have not been updated
//! @details
//! Force update of the global 32b counters and compute position and speed
/***************************************************************************/

bool process_enc( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp return
	bool f_ret;
	//Counter
	uint8_t cnt;
	//Temp counters
	int32_t enc_cnt[NUM_ENC];
	//Temp var
	int32_t tmp;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

		//----------------------------------------------------------------
		//	UPDATE GLOBAL COUNTERS
		//----------------------------------------------------------------
		
	//Failure can happen because the encoder decoder detected concurrent access to global vars. Just retry will do because access will be complete by then
	//Initialize 
	f_ret = true;
	cnt = 0;
	//While global counters have not been updated
	while ((f_ret == true) && (cnt < ENC_UPDATE_RETRY))
	{
		//Force update of global counters and return the encoder readings
		f_ret = get_enc_cnt( enc_cnt );	
		//Count attempts
		cnt++;
	}
	//If: alloted times to retry have been exceeded
	if (cnt >= ENC_UPDATE_RETRY)
	{
		//Report error
		report_error( Error_code::ERR_ENC_RETRY );
		//Failed to update encoder
		return true;
	}
	
		//----------------------------------------------------------------
		//	COMPUTE POSITION AND SPEED
		//----------------------------------------------------------------
		//	Speed is the difference between new and old position reading
	
	//For: each encoder channel
	for (cnt = 0;cnt< NUM_ENC;cnt++)
	{
		//Fetch previous position
		tmp = g_enc_pos[cnt];
		//Compute speed. Clip to int16_t
		tmp = AT_SAT( enc_cnt[cnt] -tmp, INT16_MAX, INT16_MIN );
		//save speed
		g_enc_spd[cnt] = tmp;
		//Save position
		g_enc_pos[cnt] = enc_cnt[cnt];
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: process_enc | void

