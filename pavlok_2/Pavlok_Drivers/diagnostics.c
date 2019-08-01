//-------------------------------------------------------------------
//  Routines for development diagnostics, performance measurement, etc.
//

#include "nrf_drv_systick.h"
#include "nrf_systick.h"

#include "diagnostics.h"

DiagInfo_t gDiag;



//-------------------------------------
//
uint32_t diag_ticks(void) {
    return nrf_systick_val_get();
}


//-------------------------------------
//  Calculate and return delta since given ticks.
uint32_t diag_delta(uint32_t start) {
    // systick value counts down so subtraction reversed from the expected
    uint32_t delta = start - diag_ticks();

    // value is 24-bit so ignore upper bits
    delta &= 0xffffff;

    return (delta + 32) / 64;
}


//-------------------------------------
//
void diag_init(void) {
    nrf_drv_systick_init();

    gDiag.start = diag_ticks();
}

