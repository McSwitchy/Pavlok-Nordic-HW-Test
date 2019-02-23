#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define UART_TX_BUF_SIZE        (0x100)
#define UART_RX_BUF_SIZE        (0x100)
#define UART_DEBUG_ENABLE 			(0)

extern char debugBuffer[UART_TX_BUF_SIZE];
void init_uart(void);
void debug_uart(void);

#define UART_DEBUG(x, ...) do { if (UART_DEBUG_ENABLE) { sprintf(debugBuffer, x, ##__VA_ARGS__); debug_uart(); } } while (0)


/** ----------------------------------------------------------------------
**  The following DEBUG? macros can be used for both integer (DEBUGI hex)
**  and string (DBUGS) messages to the uart port.
**  The user is encouraged to copy and make use of these for functional
**  area debug.  To do this requires adding an integer for the level of
**  debug and changing the DEBUG? macros to something like PULSE_DEBUG.
**
**  ----------------------------------------------------------------------
*/
#ifdef APP_DEBUG
extern int32_t     app_debugi_on;
extern int32_t     app_debugs_on;
extern int32_t     trace_debug_on;
#endif

#ifdef APP_DEBUG
/** ----------------------------------------------------------------------
**  pulse_debugi_on levels
**  1 error code returns
**  2 ls state
**  3 ls msg type
**  4 ls msg parms
**  5 ls ???
**  ----------------------------------------------------------------------
 */
#define DEBUGI_APP_100(parmString, parmInt)                                 \
{                                                                       \
  if (1 <= app_debugi_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s 0x%X ", __FILE__,__LINE__, parmString, parmInt);     \
  }                                                                     \
}

#define DEBUGI_APP_200(parmString, parmInt)                                 \
{                                                                       \
  if (2 <= app_debugi_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s 0x%X ", __FILE__,__LINE__, parmString, parmInt);     \
  }                                                                     \
}

#define DEBUGI_APP_300(parmString, parmInt)                                 \
{                                                                       \
  if (3 <= app_debugi_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s 0x%X ", __FILE__,__LINE__, parmString, parmInt);     \
 }                                                                     \
}

#define DEBUGI_APP_400(parmString, parmInt)                                 \
{                                                                       \
  if (4 <= app_debugi_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s 0x%X ", __FILE__,__LINE__, parmString, parmInt);     \
 }                                                                     \
}

#define DEBUGI_APP_500(parmString, parmInt)                                 \
{                                                                       \
  if (5 <= app_debugi_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s 0x%X ", __FILE__,__LINE__, parmString, parmInt);     \
 }                                                                     \
}

#else
#define DEBUGI_APP_100(parmString, parmInt);
#define DEBUGI_APP_200(parmString, parmInt);
#define DEBUGI_APP_300(parmString, parmInt);
#define DEBUGI_APP_400(parmString, parmInt);
#define DEBUGI_APP_500(parmString, parmInt);
#endif // APP_DEBUG
/** ----------------------------------------------------------------------
**  pulse_debugi_on levels
**  1 error code returns
**  2 ls state
**  3 ls msg type
**  4 ls msg parms
**  5 ls ???
**  ----------------------------------------------------------------------
 */
#ifdef APP_DEBUG

#define CC_TRACE()			\
{                                                                       \
  if (0 != trace_debug_on)                                                   \
  {                                                                     \
    SEGGER_RTT_printf(0, "\r\n%s:%d", __FILE__,__LINE__);     \
  }                                                                     \
}

#define FUNC_TRACE()			\
{                                                                       \
  if (0 != trace_debug_on)                                                   \
  {                                                                     \
    SEGGER_RTT_printf(0, "\r\n%s:%d", __FUNCTION__,__LINE__);     \
  }                                                                     \
}


#define DEBUGS_APP_100(parmString)                                 \
{                                                                       \
  if (1 <= app_debugs_on)                                                   \
  {                                                                     \
    SEGGER_RTT_printf(0, "\r\n%s:%d %s", __FILE__,__LINE__, parmString);     \
  }                                                                     \
}

#define DEBUGS_APP_200(parmString)                                 \
{                                                                       \
  if (2 <= app_debugs_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s", __FILE__,__LINE__, parmString);     \
  }                                                                     \
}

#define DEBUGS_APP_300(parmString)                                 \
{                                                                       \
  if (3 <= app_debugs_on)                                                   \
  {                                                                     \
		  SEGGER_RTT_printf(0, "\r\n%s:%d %s", __FILE__,__LINE__, parmString);     \
 }                                                                     \
}

#define DEBUGS_APP_400(parmString)                                 \
{                                                                       \
  if (4 <= app_debugs_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s", __FILE__,__LINE__, parmString);     \
  }                                                                     \
}

#define DEBUGS_APP_500(parmString)                                 \
{                                                                       \
  if (5 <= app_debugs_on)                                                   \
  {                                                                     \
		   SEGGER_RTT_printf(0, "\r\n%s:%d %s", __FILE__,__LINE__, parmString);     \
 }                                                                     \
}

#else
#define CC_TRACE();
#define FUNC_TRACE();
#define DEBUGS_APP_100(parmString);
#define DEBUGS_APP_200(parmString);
#define DEBUGS_APP_300(parmString);
#define DEBUGS_APP_400(parmString);
#define DEBUGS_APP_500(parmString);


#endif /* DEBUGX*/




#endif
