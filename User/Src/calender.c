/********************************************************************************
	*	@file     calender.c
	* @author   Depth
  * @version  V1.0.0
  * @date     12-May-2014
	* @Emial    jinlu12@sina.com
	* @brief    calender module based on app timer
	******************************************************************************/
#include "calender.h"


/*********************************************************************
 * MACROS
 */

#define	YearLength(yr)	(IsLeapYear(yr) ? 366 : 365)

/*********************************************************************
 * CONSTANTS
 */

// (MAXCALCTICKS * 5) + (max remainder) must be <= (uint16_t max),
// so: (13105 * 5) + 7 <= 65535
#define MAXCALCTICKS  ((uint16_t)(13105))

#define	BEGYEAR	        2000     // UTC started at 00:00:00 January 1, 2000

#define	DAY             86400UL  // 24 hours * 60 minutes * 60 seconds


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint16_t ll_McuPrecisionCount(void);

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint16_t previousLLTimerTick = 0;
static uint16_t remUsTicks = 0;
static uint16_t timeMSec = 0;

// number of seconds since   2010.1.1.  00:00:00
UTCTime TimeSeconds = 0;

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
static uint8_t monthLength( uint8_t lpyr, uint8_t mon );

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      Set_Clock
 *
 * @brief   Set the new time.  This will only set the seconds portion
 *          of time and doesn't change the factional second counter.
 *
 * @param   newTime - number of seconds since 0 hrs, 0 minutes,
 *                    0 seconds, on the 1st of January 2000 UTC
 *
 * @return  none
 */
void Set_Clock( UTCTime newTime )
{
  TimeSeconds = newTime;
}

/*********************************************************************
 * @fn      Get_Clock
 *
 * @brief   Gets the current time.  This will only return the seconds
 *          portion of time and doesn't include the factional second
 *          counter.
 *
 * @param   none
 *
 * @return  number of seconds since 0 hrs, 0 minutes, 0 seconds,
 *          on the 1st of January 2000 UTC
 */
UTCTime Get_Clock( void )
{
  return ( TimeSeconds );
}

/*********************************************************************
 * @fn      ConvertUTCTime
 *
 * @brief   Converts UTCTime to UTCTimeStruct
 *
 * @param   tm - pointer to breakdown struct
 *
 * @param   secTime - number of seconds since 0 hrs, 0 minutes,
 *          0 seconds, on the 1st of January 2000 UTC
 *
 * @return  none
 */
void ConvertUTCTime( UTCTimeStruct *tm, UTCTime secTime )
{
  // calculate the time less than a day - hours, minutes, seconds
  {
    uint32_t day = secTime % DAY;
    tm->seconds = day % 60UL;
    tm->minutes = (day % 3600UL) / 60UL;
    tm->hour = day / 3600UL;
  }

  // Fill in the calendar - day, month, year
  {
    uint16_t numDays = secTime / DAY;
    tm->year = BEGYEAR;
    while ( numDays > YearLength( tm->year ) )
    {
      numDays -= YearLength( tm->year );
      tm->year++;
    }

    tm->month = 0;
    while ( numDays > monthLength( IsLeapYear( tm->year ), tm->month ) )
    {
      numDays -= monthLength( IsLeapYear( tm->year ), tm->month );
      tm->month++;
    }
    tm->month++;
    tm->day = numDays;
	tm->day++;
  }
}

/*********************************************************************
 * @fn      monthLength
 *
 * @param   lpyr - 1 for leap year, 0 if not
 *
 * @param   mon - 0 - 11 (jan - dec)
 *
 * @return  number of days in specified month
 */
static uint8_t monthLength( uint8_t lpyr, uint8_t mon )
{
  uint8_t days = 31;

  if ( mon == 1 ) // feb
  {
    days = ( 28 + lpyr );
  }
  else
  {
    if ( mon > 6 ) // aug-dec
    {
      mon--;
    }

    if ( mon & 1 )
    {
      days = 30;
    }
  }

  return ( days );
}

/*********************************************************************
 * @fn      ConvertUTCSecs
 *
 * @brief   Converts a UTCTimeStruct to UTCTime
 *
 * @param   tm - pointer to provided struct
 *
 * @return  number of seconds since 00:00:00 on 01/01/2000 (UTC)
 */
UTCTime ConvertUTCSecs( UTCTimeStruct *tm )
{
  uint32_t seconds;

  /* Seconds for the partial day */
  seconds = (((tm->hour * 60UL) + tm->minutes) * 60UL) + tm->seconds;

  /* Account for previous complete days */
  {
    /* Start with complete days in current month */
    uint16_t days = tm->day;

    /* Next, complete months in current year */
    {
      int8_t month = tm->month - 1;//
      while ( --month >= 0 )
      {
        days += monthLength( IsLeapYear( tm->year ), month );
      }
    }

    /* Next, complete years before current year */
    {
      uint16_t year = tm->year;
      while ( --year >= BEGYEAR )
      {
        days += YearLength( year );
      }
    }

    /* Add total seconds before partial day */
    seconds += (days * DAY);
  }

  return ( seconds );
}

