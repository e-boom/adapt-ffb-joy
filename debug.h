/*
  Force Feedback Joystick
  Basic debugging utilities.

  This code is for Microsoft Sidewinder Force Feedback Pro joystick
  with some room for additional extra controls.

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  MIT License.

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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "main.h"

// If below are defined, code for respective debug target is included into build
//#define DEBUG_ENABLE_UART
//#define DEBUG_ENABLE_USB

// Without a debug target every Log* call and DoDebug() block below compiles to nothing
#if defined(DEBUG_ENABLE_UART) || defined(DEBUG_ENABLE_USB)
#define DEBUG_ENABLED 1
#else
#define DEBUG_ENABLED 0
#endif

// Method of debugging, bits of gDebugMode
#define DEBUG_TO_NONE	0	// Disable sending debug data
#define DEBUG_TO_UART	1	// Debug data sent to UART-out
#define DEBUG_TO_USB	2	// Debug data sent to USB COM-port - enables basic level debugging
#define DEBUG_DETAIL	4	// Include additional details to debug data

extern volatile uint8_t gDebugMode;

#define DEBUG_BUFFER_SIZE 512

#if DEBUG_ENABLED

// Returns true if debug settings contain all of the given attributes
// (see above constants DEBUG_xxx).
bool DoDebug(const uint8_t type);

// Debugging utilities

// The basic debug data sending method used by all other methods.
// Implement this to send debug data to desired destination.
void LogSendByte(uint8_t data);

// Send out the given null terminated text
void LogText(const char *text);
void LogTextLf(const char *text);	// Adds linefeed
void LogTextP(const char *text);	// From program memory
void LogTextLfP(const char *text);	// From program memory, adds linefeed

// Send out the given binary data
void LogBinary(const void *data, uint16_t len);
void LogBinaryLf(const void *data, uint16_t len);	// Adds linefeed

// Send out data with a prefix of text and an integer
void LogData(const char *text, uint8_t reportId, const void *data, uint16_t len);
void LogDataLf(const char *text, uint8_t reportId, const void *data, uint16_t len);	// Adds linefeed
void LogDataP(const char *text, uint8_t reportId, const void *data, uint16_t len);	// Text from program memory
void LogDataLfP(const char *text, uint8_t reportId, const void *data, uint16_t len);	// Text from program memory, adds linefeed

// Log all reports found in the given data (may have one or more)
// The <text> and <reportSizeArray> must point to program memory.
void LogReport(const char *text, const uint8_t *reportSizeArray, uint8_t *data, uint16_t len);

// Debugging utils for USB-serial debugging
void FlushDebugBuffer(void);

#else // !DEBUG_ENABLED

// Forced inline so the call and its string argument are dropped whatever the inliner decides
static inline bool ATTR_ALWAYS_INLINE DoDebug(const uint8_t type) { (void)type; return false; }

static inline void ATTR_ALWAYS_INLINE LogSendByte(uint8_t data) { (void)data; }

static inline void ATTR_ALWAYS_INLINE LogText(const char *text) { (void)text; }
static inline void ATTR_ALWAYS_INLINE LogTextLf(const char *text) { (void)text; }
static inline void ATTR_ALWAYS_INLINE LogTextP(const char *text) { (void)text; }
static inline void ATTR_ALWAYS_INLINE LogTextLfP(const char *text) { (void)text; }

static inline void ATTR_ALWAYS_INLINE LogBinary(const void *data, uint16_t len) { (void)data; (void)len; }
static inline void ATTR_ALWAYS_INLINE LogBinaryLf(const void *data, uint16_t len) { (void)data; (void)len; }

static inline void ATTR_ALWAYS_INLINE LogData(const char *text, uint8_t reportId, const void *data, uint16_t len)
	{ (void)text; (void)reportId; (void)data; (void)len; }
static inline void ATTR_ALWAYS_INLINE LogDataLf(const char *text, uint8_t reportId, const void *data, uint16_t len)
	{ (void)text; (void)reportId; (void)data; (void)len; }
static inline void ATTR_ALWAYS_INLINE LogDataP(const char *text, uint8_t reportId, const void *data, uint16_t len)
	{ (void)text; (void)reportId; (void)data; (void)len; }
static inline void ATTR_ALWAYS_INLINE LogDataLfP(const char *text, uint8_t reportId, const void *data, uint16_t len)
	{ (void)text; (void)reportId; (void)data; (void)len; }

static inline void ATTR_ALWAYS_INLINE LogReport(const char *text, const uint8_t *reportSizeArray, uint8_t *data, uint16_t len)
	{ (void)text; (void)reportSizeArray; (void)data; (void)len; }

static inline void ATTR_ALWAYS_INLINE FlushDebugBuffer(void) { }

#endif // DEBUG_ENABLED

#endif // _DEBUG_H_
