#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

//samg55_xplained_pro.h

#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS  0 /* Not Mounted */
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
/*TBD startup time needs to be adjusted according to measurements */
#define BOARD_OSC_STARTUP_US      15625

#endif