#include <fract2float_conv.h>

#include "osc.h"

//----------------
//--- static vars
/// assume all oscs have the same samplerate
/// phase increment at 1hz:
static fix16 ips;
/// phase increment limits
static fix16 incMin;
static fix16 incMax;
static u32 incRange;
// multiplier to map w-> max shape
static u32 shapeLimMul;

//------------------
//---- static functions


// calculate modulated and bandlimited waveshape
static inline void osc_calc_shape(osc* osc) {
  fract32 sm; // mod shape
  fract32 sl; // shape limit given current freq

  // add modulation
  sm = add_fr1x32(osc->shape, mul_fr1x32x32(osc->wmIn, osc->wmAmt) );
  
  //- hacky magic formula for pseudo-bandlimiting:
  //- with maximal bandlimiting, want to limit shape to a function of unit freq
  //- max freq = min shape, min freq = max shape
  // :
  // map phase increment to [0,1] fract32
  sl  = ((u32)(osc->inc) - incRange) * shapelimMul;
  // invert [0,1] to [1,0]
  sl = (s32)sl * -1 + 0x7fffffff;
  // limit
  if(sl < sm) {
    sm = dsp_lerp32(sm, sl, osc->bandLim);
  }
  osc->shapeMod = sm;
}

// calculate phase incremnet
static inline void osc_calc_inc( osc* osc) {
  filter_1p_lo_in( &(osc->lpInc), fix16_mul(osc->ratio, fix16_mul(osc->hz, ips)) );
}

// calculate phase
static inline void osc_calc_pm(osc* osc) {
  osc->idxMod = fix16_add( osc->idx, 
			   fix16_mul( FRACT_FIX16( mul_fr1x32x32( 
								 osc->pmIn, 
								 osc->pmAmt 
								  ) ) ) );
}

// lookup 
static inline fract32 osc_lookup(osc* osc) {
  u32 idxA = osc->shapeMod >> WAVE_TAB_RSHIFT;
  u32 idxB = idxA + 1;
  
  fract32 mul = (osc->shapeMod & WAVE_TAB_MASK) << WAVE_TAB_LSHIFT;
  fract32 mulInv = sub_fr1x32(FR32_MAX, mul);
  
  return add_fr1x32( 
		    mult_fr1x32x32(table_lookup_idx( (fract32*)osc->tab[idxA], 
						     WAVE_TAB_SIZE, 
						     osc->idxMod
						     ), mulInv ),
		    mult_fr1x32x32(table_lookup_idx( (fract32*)osc->tab[idxB],
						     WAVE_TAB_SIZE,
						     osc->idxMod 
						     ), mul 
				   ) );
}

// advance phase
static inline void osc_advance(osc* osc) {
  
}
 

//----------------
//--- extern funcs

// initialize given table data and samplerate
void osc_init(osc* osc, fract32** tab, u32 sr) {
  osc->tab = tab;
  ips = fix16_from_float( (f32)WAVE_TAB_SIZE / (f32)sr );
  incMin = fix16_mul(ips, OSC_HZ_MIN);
  incMax = fix16_mul(ips, OSC_HZ_MAX);
  incRange = (u32)incMax - (u32)incMin;
  shapeLimMul = 0x7fffffff / incRange;
}

// set waveshape (table)
void osc_set_shape(osc* osc, fract32 shape) {
  //  osc->shape = shape;
  //  osc_calc_shape(osc);
  filter_1p_lo_in( &(osc->lpShape, shape) );
}

// set base frequency in hz
void osc_set_hz(osc* osc, fix16 hz) {
  osc->hz = hz;
  osc_calc_inc(osc);
}

// set fine-tuning ratio
void osc_set_tune(osc* osc, fix16 ratio) {
  osc->ratio = ratio;
  osc_calc_inc(osc);
}

// phase modulation amount
void osc_set_pm(osc* osc, fract32 amt) {
  filter_1p_lo_in( &(osc->lpPm), amt);
}

// shape modulation amount
void osc_set_wm(osc* osc, fract32 amt) {
  filter_1p_lo_in( &(osc->lpWm), amt);
}

// phase modulation input
void osc_pm_in(osc* osc, fract32 val) {
  osc->pmIn = val;
}

// shape modulation input
void osc_wm_in(osc* osc, fract32 val) {
  osc->wmIn = val;
}


// set bandlimiting
void osc_set_bl(osc* osc, fract32 bl) {
  osc->bandLimit = bl;
}

// get next value
fract32 osc_next(void) {
  /// update param smoothers
  osc->inc = filter_1p_lo_next( &(osc->lpInc) );
  osc->shape = filter_1p_lo_next( &(osc->lpShape) );
  osc->pmAmt = filter_1p_lo_next( &(osc->lpShape) );
  osc->wmAmt = filter_1p_lo_next( &(osc->lpShape) );
  
  // calculate waveshape
  osc_calc_shape(osc);

  // calculate phase
  osc_calc_pm(osc);

  // lookup 
  return osc_lookup(osc);
}
