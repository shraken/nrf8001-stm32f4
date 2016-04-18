/**
 * @description Global definitions and enumerations
 */

#ifndef _GLOBALS_H
#define _GLOBALS_H

#define MAX_STR_LEN 128
#define DEFAULT_WAIT_TIME 20000000u
#define DEFAULT_PAUSE_TIME 20000000u

// error code used globally for all routines
typedef enum _error_type
{
    E_SUCCESS = 0,
    E_BUSY = -1,
    E_IDLE = -2,
    E_ILLEGAL = -3,
    E_CHK_PATT_FAIL = -4,
    E_HWFAULT_FAIL = -5,
    E_GENERAL_FAIL = -6
} error_type;

#endif