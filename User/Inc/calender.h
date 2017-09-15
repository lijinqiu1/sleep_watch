/********************************************************************************
	*	@file     calender.h
	* @author   Depth
  * @version  V1.0.0
  * @date     12-May-2014
	* @Emial    jinlu12@sina.com
	* @brief    head file for calender.c
	******************************************************************************/

#ifndef CALENER_H
#define CALENER_H


#ifdef __cplusplus
extern "C"
{
#endif


#include <stdint.h>
#include "app_timer.h"

/*********************************************************************
 * MACROS
 */


#define	IsLeapYear(yr)	(!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))

/*********************************************************************
 * TYPEDEFS
 */

// number of seconds since 0 hrs, 0 minutes, 0 seconds, on the
// 1st of January 2000 UTC
typedef uint32_t UTCTime;

extern UTCTime TimeSeconds;

// To be used with
typedef struct
{
  uint8_t seconds;  // 0-59
  uint8_t minutes;  // 0-59
  uint8_t hour;     // 0-23
  uint8_t day;      // 0-30
  uint8_t month;    // 0-11
  uint16_t year;    // 2000+
} UTCTimeStruct;

/*********************************************************************
 * FUNCTIONS
 */

  /*
   * Updates the clock and Timers from the MAC 320us timer tick.
   */
  extern void TimeUpdate( void );

  /*
   * Set the new time.  This will only set the seconds portion
   * of time and doesn't change the factional second counter.
   *     newTime - number of seconds since 0 hrs, 0 minutes,
   *               0 seconds, on the 1st of January 2000 UTC
   */
  extern void Set_Clock( UTCTime newTime );

  /*
   * Gets the current time.  This will only return the seconds
   * portion of time and doesn't include the factional second counter.
   *     returns: number of seconds since 0 hrs, 0 minutes,
   *              0 seconds, on the 1st of January 2000 UTC
   */
  extern UTCTime Get_Clock( void );

  /*
   * Converts UTCTime to UTCTimeStruct
   *
   * secTime - number of seconds since 0 hrs, 0 minutes,
   *          0 seconds, on the 1st of January 2000 UTC
   * tm - pointer to breakdown struct
   */
  extern void ConvertUTCTime( UTCTimeStruct *tm, UTCTime secTime );

  /*
   * Converts UTCTimeStruct to UTCTime (seconds since 00:00:00 01/01/2000)
   *
   * tm - pointer to UTC time struct
   */
  extern UTCTime ConvertUTCSecs( UTCTimeStruct *tm );

  /*
   * Update/Adjust the clock and timers
   * Msec - elapsed time in milli seconds
   */
  extern void AdjustTimer( uint32_t Msec );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CALENDER_H */
