#include <compiler.h>
#include <status_codes.h>

#include "conf_clock.h"

#include <sam.h> //Include SAMG55 CMSIS

#include "interrupt_sam_nvic.h"

//Include SAM peripheral drivers.
#include "adc2.h"
#include "efc.h"
#include "flexcom.h"
#include "matrix.h"
#include "pio.h"
#include "pmc.h"
#include "sleep.h"
#include "supc.h"
#include "tc.h"
#include "udp_device.h"
#include "usart.h"
#include "wdt.h"

#include "osc.h"
#include "pll.h"
#include "sysclk.h"