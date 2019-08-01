#ifndef PAV_DIAGNOSTICS_H
#define PAV_DIAGNOSTICS_H
//-------------------------------------------------------------------
//  Routines for development diagnostics, performance measurement, etc.
//

//-------------------------------------
//  Diagnostics
//
typedef volatile struct
{
    uint32_t        start;
} DiagInfo_t;

extern DiagInfo_t  gDiag;

#define SYSTICK_DIFF(a, b)      ((((a) - (b)) & 0xffffffL))


void        diag_init(void);
uint32_t    diag_ticks(void);
uint32_t    diag_delta(uint32_t start);


#endif
