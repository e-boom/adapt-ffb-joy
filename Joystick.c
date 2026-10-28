/*
  Force Feedback Joystick
  Joystick model specific code for handling joystick input data.
  This code is for Microsoft Sidewinder Force Feedback Pro joystick.

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2013  Saku Kekkonen - Modifications for FF Wheel support
  MIT License.

  Code marked with "3DPVert" is from Detlef "Grendel" Mueller's
  3DPVert-project and is distributed with artistic/GPL license.

  Some code is based on LUFA Library, for which uses MIT license:

  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/


#include <avr/pgmspace.h>

#include "3DPro.h"
#include "Joystick.h"
#include "ffb.h"
#include "usb_hid.h"
#include "debug.h"

//#define USE_FAKE_JOYSTICK

const uint8_t INPUT_REPORTID_ALL = 0xFF;


/** Configures the board hardware and chip peripherals for the joystick's functionality. */
void Joystick_Init(void)
	{
    // Initialize..

    init_hw() ;					// hardware and wait for the stick

    SetTMPS( 1, 64 ) ;				// Set T1 prescaler to /64
    SetTMPS( 0, 1024 ) ;			// Set T0 prescaler to / 1024 for delay

	WaitMs(1000);

	// Force feedback
	FfbInitMidi();

	// ADC for extra controls
	DDRF = 0; // all inputs
	PORTF |= 0x33;	// pull-ups on the analog pins: without pots fitted they would float and jitter
	DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC4D) | (1 << ADC5D);	// digital input buffers off on the analog pins: less power and noise

	// Init and enable ADC
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX |= (1 << REFS0);		// ADC reference := Vcc
//	ADCSRA |= (1 << ADFR); 		// ADATE
//	ADCSRA |= (1 << ADATE); 	// free/continous mode
	ADMUX |= (1 << ADLAR); 		// 8-bit resolution to HIGH
	ADCSRA |= (1 << ADEN); 		// Enable ADC
	ADCSRA |= (1 << ADIE);  	// Enable ADC Interrupt 

	ADCSRA |= (1 << ADSC); 		// Go ADC
	}

/** Always reports the stick as connected: init_hw() has already waited for it. */
int Joystick_Connect(void)
	{
	return 1;
	}

typedef struct
	{
	uint8_t sampledChannel;
	uint8_t	trim1, trim2, pedal1, pedal2;
	} AddedControls_ADC_t;

static volatile AddedControls_ADC_t added_controls_adc;

static volatile uint8_t ADC_is_ready = 0;


// Jitter filter for the rudder axes: the FFP's own twist (Rz) and the two
// pedals on ADC4/ADC5 that make up the Slider.
//
// Running the force feedback motors adds electrical noise to these axes, which
// shows up as jitter. Each new sample goes through three stages:
//	1. median of the last 3 samples: drops single-sample spikes
//	2. moving average: smooths the remaining noise, keeping fractions of a count
//	3. hysteresis: the reported value only changes once the average has moved
//	   clearly away from it, so it does not flicker between two neighbours
//
// RUDDER_FILTER: 0 passes samples through unfiltered.
// RUDDER_FILTER_SMOOTHING: a new sample weighs 1/2^n in the average, which then
//	lags about 2^n - 1 samples behind. Higher is calmer but slower to follow
//	the stick. 0 = no averaging.
// RUDDER_FILTER_HYSTERESIS: how far, in percent of one count, the average has to
//	move away from the reported value before that changes. 50 = no hysteresis.
//	Keep it below 100, or the axis can stay stuck one count short of where it is.

#define RUDDER_FILTER				1
#define RUDDER_FILTER_SMOOTHING		3
#define RUDDER_FILTER_HYSTERESIS	85

#define AXIS_FILTER_FRACTION_BITS	6	// the average is kept in 1/64 counts
#define AXIS_FILTER_HYSTERESIS		((RUDDER_FILTER_HYSTERESIS << AXIS_FILTER_FRACTION_BITS) / 100)

typedef struct
	{
	uint8_t	history[2];	// the previous two samples, for the median
	int16_t	average;	// in 1/64 counts
	uint8_t	output;		// the value currently reported
	uint8_t	primed;		// set once the state has been seeded from a first sample
	} AxisFilter_t;

static AxisFilter_t twist_filter, pedal1_filter, pedal2_filter;

// Runs one new sample through the filter and returns the value to report.
static uint8_t AxisFilter_Update(AxisFilter_t* filter, uint8_t sample)
	{
	if (!RUDDER_FILTER)
		return sample;

	// Seed the state from the first sample, so the reports do not ramp up from zero after plug-in
	if (!filter->primed)
		{
		filter->history[0] = filter->history[1] = sample;
		filter->average = (int16_t) sample << AXIS_FILTER_FRACTION_BITS;
		filter->output = sample;
		filter->primed = 1;
		return sample;
		}

	// 1. Median of this sample and the previous two
	uint8_t low = filter->history[0], high = filter->history[1], median;
	if (low > high)
		{
		low = filter->history[1];
		high = filter->history[0];
		}
	if (sample < low)
		median = low;
	else if (sample > high)
		median = high;
	else
		median = sample;
	filter->history[0] = filter->history[1];
	filter->history[1] = sample;

	// 2. Moving average. It always moves at least 1/64 count, so that it settles
	// exactly on a steady input instead of stalling just short of it.
	int16_t delta = ((int16_t) median << AXIS_FILTER_FRACTION_BITS) - filter->average;
	int16_t step = delta / (1 << RUDDER_FILTER_SMOOTHING);
	if (step == 0)
		step = (delta > 0) - (delta < 0);
	filter->average += step;

	// 3. Hysteresis
	int16_t distance = filter->average - ((int16_t) filter->output << AXIS_FILTER_FRACTION_BITS);
	if (distance > AXIS_FILTER_HYSTERESIS || distance < -AXIS_FILTER_HYSTERESIS)
		filter->output = (filter->average + (1 << (AXIS_FILTER_FRACTION_BITS - 1))) >> AXIS_FILTER_FRACTION_BITS;

	return filter->output;
	}



// Twist drift correction for the FFP.
//
// The gimbal turns the handle a little when the stick is tilted, so the raw twist
// reading changes with X and Y even with the motors off and the handle untouched,
// and so do the readings at its two stops. Three tables hold the raw twist on a
// 5 x 5 grid of stick positions: with the handle untouched, and at its left and
// right stop. In between, each is interpolated bilinearly. The reported twist is
// then stretched so that, at every stick position, the untouched handle reads 32
// and the stops read 0 and 63.
//
// TWIST_DRIFT_CORRECTION: 0 reports the twist uncorrected.
// twist_grid_x/_y: the X and Y values of the tables' columns and rows. The middle
//	ones are where the stick rested in the recording (multi_controller raw / 64).
// twist_low/_centre/_high: raw twist in quarter counts (0..252 for 0..63); rows
//	forward (Y negative) to back, columns left to right. From the user's stick on
//	2026-10-28: the centre fitted by tools/fit_twist_tables.py to a recording of
//	the stick moved round without twisting; the stops from two recordings twisting
//	to both ends (the left stop at the forward-left and back-right corners held
//	exactly in the corner, one count added so the end is always reached), carried
//	over from a 3 x 3 table.

#define TWIST_DRIFT_CORRECTION		1
#define TWIST_GRID					5

static const int16_t twist_grid_x[TWIST_GRID] PROGMEM = { -512, -294, -76, 218, 511 };
static const int16_t twist_grid_y[TWIST_GRID] PROGMEM = { -512, -249,  14, 262, 511 };

static const uint8_t twist_low[TWIST_GRID][TWIST_GRID] PROGMEM =
	{
	{  40,  20,   0,   0,   0 },	// forward
	{  31,  15,   0,   0,   0 },
	{  21,  10,   0,   0,   0 },	// rest
	{  20,  10,   0,  12,  24 },
	{  20,  10,   0,  26,  52 },	// back
	};

static const uint8_t twist_centre[TWIST_GRID][TWIST_GRID] PROGMEM =
	{
	{ 124, 126, 134, 129, 158 },	// forward
	{ 109, 135, 133, 138, 155 },
	{ 111, 134, 136, 123, 160 },	// rest
	{ 130, 137, 140, 125, 148 },
	{ 157, 162, 139, 124, 129 },	// back
	};

static const uint8_t twist_high[TWIST_GRID][TWIST_GRID] PROGMEM =
	{
	{ 252, 242, 232, 242, 252 },	// forward
	{ 252, 247, 241, 246, 250 },
	{ 252, 251, 251, 250, 248 },	// rest
	{ 252, 252, 252, 251, 250 },
	{ 252, 252, 252, 252, 252 },	// back
	};

// Returns the index of the grid cell that value falls in: 0..TWIST_GRID-2.
static uint8_t TwistCell(const int16_t* grid, int16_t value)
	{
	uint8_t cell = TWIST_GRID - 2;
	while (cell > 0 && value < (int16_t) pgm_read_word(&grid[cell]))
		cell--;
	return cell;
	}

// Returns the table's value at stick position (x, y), in 1/256 counts.
static int16_t TwistTable(const uint8_t table[TWIST_GRID][TWIST_GRID], int16_t x, int16_t y)
	{
	uint8_t col = TwistCell(twist_grid_x, x);
	uint8_t row = TwistCell(twist_grid_y, y);
	int16_t x0 = pgm_read_word(&twist_grid_x[col]), x1 = pgm_read_word(&twist_grid_x[col + 1]);
	int16_t y0 = pgm_read_word(&twist_grid_y[row]), y1 = pgm_read_word(&twist_grid_y[row + 1]);

	// Position inside the cell, in 1/256
	int16_t fx = ((int32_t) (x - x0) << 8) / (x1 - x0);
	int16_t fy = ((int32_t) (y - y0) << 8) / (y1 - y0);

	int32_t top    = (int32_t) pgm_read_byte(&table[row][col]) * (256 - fx) + (int32_t) pgm_read_byte(&table[row][col + 1]) * fx;
	int32_t bottom = (int32_t) pgm_read_byte(&table[row + 1][col]) * (256 - fx) + (int32_t) pgm_read_byte(&table[row + 1][col + 1]) * fx;

	// Quarter counts times 65536, to 1/256 counts
	return (top * (256 - fy) + bottom * fy + 512) >> 10;
	}

/** Fills the given HID report data structure with the next HID report to send to the host.
 *
 *  \param[out] outReportData  Pointer to a HID report data structure to be filled
 *
 *  \return Always true: there is no change detection, every call yields a report
 */
int Joystick_CreateInputReport(uint8_t inReportId, USB_JoystickReport_Data_t* const outReportData)
	{
	// Read the data from the FFP-joystick

#ifndef USE_FAKE_JOYSTICK
	// Code from 3DPVert begins-->
	SetTMPS( 0, 64 ) ;		// Set T0 prescaler to / 64 for query
	getdata();

	// -------------------------------------------------------------------------------
	// *******************************************************************************
	// 	Decode the raw FFP/PP data and store it into sw_report
	// 
	//  Input:
	// 	Pointer to start of packet to copy
	// 
	// 	FFP/PP data packet structure
	// 	============================
	// 
	// 	44444444 33333333 33222222 22221111 11111100 00000000
	// 	76543210 98765432 10987654 32109876 54321098 76543210
	// 	-------0 -------1 -------2 -------3 -------4 -------5
	// 	ppHHHHRR RRRRTTTT TTTYYYYY YYYYYXXX XXXXXXXB BBBBBBBB
	// 	  321054 32106543 21098765 43210987 65432109 87654321
	// 
	// 	USB report data structure
	// 	=========================
	// 
	// 	-------0 -------1 -------2 -------3 -------4 -------5
	// 	XXXXXXXX YYYYYYXX HHHHYYYY BBRRRRRR TBBBBBBB 00TTTTTT
	// 	76543210 54321098 32109876 21543210 09876543   654321
	// 
	// -------------------------------------------------------------------------------

	// Copy the data to the USB report
	// <--- Code from 3DPVert ends
#else
	memset(sw_report, 0, sizeof(sw_report));
#endif

	// ???? This could be done more directly by modifying the 3DPVert code
	// ???? that generates its own USB report to the abovementioned format.
	// Here we read the additional analog controls in
	// rotation for each AD-channel at a time.
	//
	// Here,
	//	ADC0 - Trim 1
	//	ADC1 - Trim 2
	//	ADC4 - Left Pedal
	//	ADC5 - Right Pedal

	if (ADC_is_ready)
		{
		ADC_is_ready = 0;

		switch (added_controls_adc.sampledChannel)
			{
			case 0:
				added_controls_adc.trim1 = ADCH;
				added_controls_adc.sampledChannel = 1;
				break;
			case 1:
				added_controls_adc.trim2 = ADCH;
				added_controls_adc.sampledChannel = 4;
				break;
			case 4:
				added_controls_adc.pedal1 = AxisFilter_Update(&pedal1_filter, ADCH);
				added_controls_adc.sampledChannel = 5;
				break;
			case 5:
				added_controls_adc.pedal2 = AxisFilter_Update(&pedal2_filter, ADCH);
				added_controls_adc.sampledChannel = 0;
				break;
			default:
				added_controls_adc.sampledChannel = 0;	// just in case
				break;
			}

		// Start AD-conversion for the next channel in the rotation
		ADMUX &= 0b11111000;
		ADMUX |= (0b111 & added_controls_adc.sampledChannel);
		ADCSRA|=(1<<ADSC);
		}

/*
5 wwwwwwww
4 aaaaaaww 
3 BAbbbbbb
2 1FLZYXRC
1 p-------
0 --------
*/
		
	// Convert the raw input data to USB report

	// The caller's report is not cleared, and the wheel fills only some fields
	memset(outReportData, 0, sizeof(*outReportData));
	outReportData->reportId = 1;	// Input report ID 1

	if (sw_id == SW_ID_FFPW)
		{
		outReportData->Button = ((sw_report[2] << 2) | (sw_report[3] >> 6)) ^ 0x00ff;
		outReportData->X = ((sw_report[4] & 0x03) << 8) + sw_report[5];
		outReportData->Y = (sw_report[4] & 0xfc) << 2;
		/* actually break for wheel */
		outReportData->Throttle = 63-(sw_report[3] & 0x3f);
		outReportData->Hat = 8;	// null state: outside the hat's logical range 0..7
		}
	else
		{
		// Convert data from Sidewinder Force Feedback Pro
		outReportData->X = sw_report[0] + ((sw_report[1] & 0x03) << 8);
		if (sw_report[1] & 0x02)
			outReportData->X |= 0xFC00u;	// sign-extend the 10-bit value
		
		
		outReportData->Y = (sw_report[1] >> 2) + ((sw_report[2] & 0x0F) << 6);
		if (sw_report[2] & 0x08)
			outReportData->Y |= 0xFC00u;	// sign-extend the 10-bit value
		outReportData->Button = ((sw_report[4] & 0x7F) << 2) + ((sw_report[3] & 0xC0) >> 6);
		outReportData->Hat = sw_report[2] >> 4;
		// The twist arrives as a signed 6-bit value, centred on the wrap from 63 to 0.
		// Shift it to 0..63 (centre 32) first, so the filter never averages across that wrap.
		int16_t twist = ((sw_report[3] & 0x3f) - 32) & 0x3f;
		if (TWIST_DRIFT_CORRECTION)
			{
			// Map the drifted centre to 32 and the drifted stops to 0 and 63, each
			// side stretched to fill its half. All in 1/256 counts.
			int16_t low    = TwistTable(twist_low, outReportData->X, outReportData->Y);
			int16_t centre = TwistTable(twist_centre, outReportData->X, outReportData->Y);
			int16_t high   = TwistTable(twist_high, outReportData->X, outReportData->Y);
			if (centre < low + 256)		// keep at least a count on either side
				centre = low + 256;
			if (high < centre + 256)
				high = centre + 256;

			int16_t raw = twist * 256;
			int32_t mapped;
			if (raw >= centre)
				mapped = 32 * 256 + (int32_t) (raw - centre) * (31 * 256) / (high - centre);
			else
				mapped = 32 * 256 - (int32_t) (centre - raw) * (32 * 256) / (centre - low);
			if (mapped < 0)				// beyond a measured stop
				mapped = 0;
			else if (mapped > 63 * 256)
				mapped = 63 * 256;
			twist = (mapped + 128) / 256;	// rounded to the nearest count
			}
		outReportData->Rz = AxisFilter_Update(&twist_filter, twist);
		outReportData->Throttle = ((sw_report[5] & 0x3f) << 1) + (sw_report[4] >> 7);
		if (sw_report[5] & 0x20)
			outReportData->Throttle |= 0b11000000;

		outReportData->Z = 0;	// not used at the moment

		// Get data from additional controls
		outReportData->Rudder = (added_controls_adc.pedal2 - added_controls_adc.pedal1) / 2 - 128;	// Combine two pedals into a single rudder
		outReportData->Rx = added_controls_adc.trim2;	// rudder trim
		outReportData->Ry = added_controls_adc.trim1;	// elevator trim
		}

	return 1;
	}


// AD-conversion-completed-interrupt handler.
// We only set a "ADC ready"-flag here so that
// the actual data can be read later when there
// is more time. Doing any heavier stuff here
// seems to cause problems reading data from FFP joystick.
ISR(ADC_vect) 
	{
	ADC_is_ready = 1;
	}
