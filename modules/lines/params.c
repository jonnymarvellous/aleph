 
// autogenerated from param_gen.scd 

#include <string.h>
#include "module.h"
#include "params.h" 

void fill_param_desc(void) {
  strcpy(gModuleData->paramDesc[eParam_delay0].label, "delay0");
  strcpy(gModuleData->paramDesc[eParam_delay0].unit, "s");
  gModuleData->paramDesc[eParam_delay0].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_delay0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_delay0].max = PARAM_BUFFER_MAX; 

  strcpy(gModuleData->paramDesc[eParam_loop0].label, "loop0");
  strcpy(gModuleData->paramDesc[eParam_loop0].unit, "s");
  gModuleData->paramDesc[eParam_loop0].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_loop0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_loop0].max = PARAM_BUFFER_MAX; 

  strcpy(gModuleData->paramDesc[eParam_rate0].label, "rate0");
  strcpy(gModuleData->paramDesc[eParam_rate0].unit, "u");
  gModuleData->paramDesc[eParam_rate0].type = PARAM_TYPE_FIX; 
  gModuleData->paramDesc[eParam_rate0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_rate0].max = 0x00010000; 

  strcpy(gModuleData->paramDesc[eParam_write0].label, "write0");
  strcpy(gModuleData->paramDesc[eParam_write0].unit, "u");
  gModuleData->paramDesc[eParam_write0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_write0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_write0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_pre0].label, "pre0");
  strcpy(gModuleData->paramDesc[eParam_pre0].unit, "u");
  gModuleData->paramDesc[eParam_pre0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_pre0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_pre0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_pos_write0].label, "pos_write0");
  strcpy(gModuleData->paramDesc[eParam_pos_write0].unit, "s");
  gModuleData->paramDesc[eParam_pos_write0].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_pos_write0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_pos_write0].max = PARAM_BUFFER_MAX; 

  strcpy(gModuleData->paramDesc[eParam_pos_read0].label, "pos_read0");
  strcpy(gModuleData->paramDesc[eParam_pos_read0].unit, "s");
  gModuleData->paramDesc[eParam_pos_read0].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_pos_read0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_pos_read0].max = PARAM_BUFFER_MAX; 

  strcpy(gModuleData->paramDesc[eParam_run_write0].label, "run_write0");
  strcpy(gModuleData->paramDesc[eParam_run_write0].unit, "s");
  gModuleData->paramDesc[eParam_run_write0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_run_write0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_run_write0].max = fix16_one; 

  strcpy(gModuleData->paramDesc[eParam_run_read0].label, "run_read0");
  strcpy(gModuleData->paramDesc[eParam_run_read0].unit, "s");
  gModuleData->paramDesc[eParam_run_read0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_run_read0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_run_read0].max = fix16_one; 

  strcpy(gModuleData->paramDesc[eParam_delay1].label, "delay1");
  strcpy(gModuleData->paramDesc[eParam_delay1].unit, "s");
  gModuleData->paramDesc[eParam_delay1].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_delay1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_delay1].max = PARAM_BUFFER_MAX; 

  strcpy(gModuleData->paramDesc[eParam_loop1].label, "loop1");
  strcpy(gModuleData->paramDesc[eParam_loop1].unit, "s");
  gModuleData->paramDesc[eParam_loop1].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_loop1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_loop1].max = PARAM_BUFFER_MAX; 

  strcpy(gModuleData->paramDesc[eParam_rate1].label, "rate1");
  strcpy(gModuleData->paramDesc[eParam_rate1].unit, "u");
  gModuleData->paramDesc[eParam_rate1].type = PARAM_TYPE_FIX; 
  gModuleData->paramDesc[eParam_rate1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_rate1].max = 0x00010000; 

  strcpy(gModuleData->paramDesc[eParam_write1].label, "write1");
  strcpy(gModuleData->paramDesc[eParam_write1].unit, "u");
  gModuleData->paramDesc[eParam_write1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_write1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_write1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_pre1].label, "pre1");
  strcpy(gModuleData->paramDesc[eParam_pre1].unit, "u");
  gModuleData->paramDesc[eParam_pre1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_pre1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_pre1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_pos_write1].label, "pos_write1");
  strcpy(gModuleData->paramDesc[eParam_pos_write1].unit, "s");
  gModuleData->paramDesc[eParam_pos_write1].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_pos_write1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_pos_write1].max = PARAM_BUFFER_MAX; 

  strcpy(gModuleData->paramDesc[eParam_pos_read1].label, "pos_read1");
  strcpy(gModuleData->paramDesc[eParam_pos_read1].unit, "s");
  gModuleData->paramDesc[eParam_pos_read1].type = PARAM_TYPE_UINT; 
  gModuleData->paramDesc[eParam_pos_read1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_pos_read1].max = PARAM_BUFFER_MAX; 


  strcpy(gModuleData->paramDesc[eParam_run_write1].label, "run_write1");
  strcpy(gModuleData->paramDesc[eParam_run_write1].unit, "s");
  gModuleData->paramDesc[eParam_run_write1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_run_write1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_run_write1].max = fix16_one; 

  strcpy(gModuleData->paramDesc[eParam_run_read1].label, "run_read1");
  strcpy(gModuleData->paramDesc[eParam_run_read1].unit, "s");
  gModuleData->paramDesc[eParam_run_read1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_run_read1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_run_read1].max = fix16_one; 


  strcpy(gModuleData->paramDesc[eParam_coeff0].label, "coeff0");
  strcpy(gModuleData->paramDesc[eParam_coeff0].unit, "coeff");
  gModuleData->paramDesc[eParam_coeff0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_coeff0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_coeff0].max = PARAM_COEFF_MAX  // FIXME 

  strcpy(gModuleData->paramDesc[eParam_rq0].label, "rq0");
  strcpy(gModuleData->paramDesc[eParam_rq0].unit, "u");
  gModuleData->paramDesc[eParam_rq0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_rq0].min = PARAM_RQ_MIN;
  gModuleData->paramDesc[eParam_rq0].max = PARAM_RQ_MAX; 

  strcpy(gModuleData->paramDesc[eParam_low0].label, "low0");
  strcpy(gModuleData->paramDesc[eParam_low0].unit, "u");
  gModuleData->paramDesc[eParam_low0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_low0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_low0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_high0].label, "high0");
  strcpy(gModuleData->paramDesc[eParam_high0].unit, "u");
  gModuleData->paramDesc[eParam_high0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_high0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_high0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_band0].label, "band0");
  strcpy(gModuleData->paramDesc[eParam_band0].unit, "u");
  gModuleData->paramDesc[eParam_band0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_band0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_band0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_notch0].label, "notch0");
  strcpy(gModuleData->paramDesc[eParam_notch0].unit, "u");
  gModuleData->paramDesc[eParam_notch0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_notch0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_notch0].max = 0x00010000;

  strcpy(gModuleData->paramDesc[eParam_fwet0].label, "fwet0");
  strcpy(gModuleData->paramDesc[eParam_fwet0].unit, "u");
  gModuleData->paramDesc[eParam_fwet0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_fwet0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_fwet0].max = FR32_MAX; 

  strcpy(gModuleData->paramDesc[eParam_fdry0].label, "fdry0");
  strcpy(gModuleData->paramDesc[eParam_fdry0].unit, "u");
  gModuleData->paramDesc[eParam_fdry0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_fdry0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_fdry0].max = FR32_MAX; 

  strcpy(gModuleData->paramDesc[eParam_coeff1].label, "coeff1");
  strcpy(gModuleData->paramDesc[eParam_coeff1].unit, "coeff");
  gModuleData->paramDesc[eParam_coeff1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_coeff1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_coeff1].max = 0x1F400000; 

  strcpy(gModuleData->paramDesc[eParam_rq1].label, "rq1");
  strcpy(gModuleData->paramDesc[eParam_rq1].unit, "u");
  gModuleData->paramDesc[eParam_rq1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_rq1].min = PARAM_RQ_MIN;
  gModuleData->paramDesc[eParam_rq1].max = PARAM_RQ_MAX; 
 
  strcpy(gModuleData->paramDesc[eParam_low1].label, "low1");
  strcpy(gModuleData->paramDesc[eParam_low1].unit, "u");
  gModuleData->paramDesc[eParam_low1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_low1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_low1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_high1].label, "high1");
  strcpy(gModuleData->paramDesc[eParam_high1].unit, "u");
  gModuleData->paramDesc[eParam_high1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_high1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_high1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_band1].label, "band1");
  strcpy(gModuleData->paramDesc[eParam_band1].unit, "u");
  gModuleData->paramDesc[eParam_band1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_band1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_band1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_notch1].label, "notch1");
  strcpy(gModuleData->paramDesc[eParam_notch1].unit, "u");
  gModuleData->paramDesc[eParam_notch1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_notch1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_notch1].max = FR32_MAX;


  strcpy(gModuleData->paramDesc[eParam_fwet1].label, "fwet1");
  strcpy(gModuleData->paramDesc[eParam_fwet1].unit, "u");
  gModuleData->paramDesc[eParam_fwet1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_fwet1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_fwet1].max = FR32_MAX; 

  strcpy(gModuleData->paramDesc[eParam_fdry1].label, "fdry1");
  strcpy(gModuleData->paramDesc[eParam_fdry1].unit, "u");
  gModuleData->paramDesc[eParam_fdry1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_fdry1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_fdry1].max = FR32_MAX; 

  strcpy(gModuleData->paramDesc[eParam_adc0_del0].label, "adc0_del0");
  strcpy(gModuleData->paramDesc[eParam_adc0_del0].unit, "u");
  gModuleData->paramDesc[eParam_adc0_del0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc0_del0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc0_del0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc0_del1].label, "adc0_del1");
  strcpy(gModuleData->paramDesc[eParam_adc0_del1].unit, "u");
  gModuleData->paramDesc[eParam_adc0_del1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc0_del1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc0_del1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc1_del0].label, "adc1_del0");
  strcpy(gModuleData->paramDesc[eParam_adc1_del0].unit, "u");
  gModuleData->paramDesc[eParam_adc1_del0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc1_del0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc1_del0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc1_del1].label, "adc1_del1");
  strcpy(gModuleData->paramDesc[eParam_adc1_del1].unit, "u");
  gModuleData->paramDesc[eParam_adc1_del1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc1_del1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc1_del1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc2_del0].label, "adc2_del0");
  strcpy(gModuleData->paramDesc[eParam_adc2_del0].unit, "u");
  gModuleData->paramDesc[eParam_adc2_del0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc2_del0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc2_del0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc2_del1].label, "adc2_del1");
  strcpy(gModuleData->paramDesc[eParam_adc2_del1].unit, "u");
  gModuleData->paramDesc[eParam_adc2_del1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc2_del1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc2_del1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc3_del0].label, "adc3_del0");
  strcpy(gModuleData->paramDesc[eParam_adc3_del0].unit, "u");
  gModuleData->paramDesc[eParam_adc3_del0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc3_del0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc3_del0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc3_del1].label, "adc3_del1");
  strcpy(gModuleData->paramDesc[eParam_adc3_del1].unit, "u");
  gModuleData->paramDesc[eParam_adc3_del1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc3_del1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc3_del1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del0_del0].label, "del0_del0");
  strcpy(gModuleData->paramDesc[eParam_del0_del0].unit, "u");
  gModuleData->paramDesc[eParam_del0_del0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del0_del0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del0_del0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del0_del1].label, "del0_del1");
  strcpy(gModuleData->paramDesc[eParam_del0_del1].unit, "u");
  gModuleData->paramDesc[eParam_del0_del1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del0_del1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del0_del1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del1_del0].label, "del1_del0");
  strcpy(gModuleData->paramDesc[eParam_del1_del0].unit, "u");
  gModuleData->paramDesc[eParam_del1_del0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del1_del0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del1_del0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del1_del1].label, "del1_del1");
  strcpy(gModuleData->paramDesc[eParam_del1_del1].unit, "u");
  gModuleData->paramDesc[eParam_del1_del1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del1_del1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del1_del1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc0_dac0].label, "adc0_dac0");
  strcpy(gModuleData->paramDesc[eParam_adc0_dac0].unit, "u");
  gModuleData->paramDesc[eParam_adc0_dac0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc0_dac0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc0_dac0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc0_dac1].label, "adc0_dac1");
  strcpy(gModuleData->paramDesc[eParam_adc0_dac1].unit, "u");
  gModuleData->paramDesc[eParam_adc0_dac1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc0_dac1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc0_dac1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc0_dac2].label, "adc0_dac2");
  strcpy(gModuleData->paramDesc[eParam_adc0_dac2].unit, "u");
  gModuleData->paramDesc[eParam_adc0_dac2].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc0_dac2].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc0_dac2].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc0_dac3].label, "adc0_dac3");
  strcpy(gModuleData->paramDesc[eParam_adc0_dac3].unit, "u");
  gModuleData->paramDesc[eParam_adc0_dac3].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc0_dac3].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc0_dac3].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc1_dac0].label, "adc1_dac0");
  strcpy(gModuleData->paramDesc[eParam_adc1_dac0].unit, "u");
  gModuleData->paramDesc[eParam_adc1_dac0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc1_dac0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc1_dac0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc1_dac1].label, "adc1_dac1");
  strcpy(gModuleData->paramDesc[eParam_adc1_dac1].unit, "u");
  gModuleData->paramDesc[eParam_adc1_dac1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc1_dac1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc1_dac1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc1_dac2].label, "adc1_dac2");
  strcpy(gModuleData->paramDesc[eParam_adc1_dac2].unit, "u");
  gModuleData->paramDesc[eParam_adc1_dac2].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc1_dac2].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc1_dac2].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc1_dac3].label, "adc1_dac3");
  strcpy(gModuleData->paramDesc[eParam_adc1_dac3].unit, "u");
  gModuleData->paramDesc[eParam_adc1_dac3].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc1_dac3].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc1_dac3].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc2_dac0].label, "adc2_dac0");
  strcpy(gModuleData->paramDesc[eParam_adc2_dac0].unit, "u");
  gModuleData->paramDesc[eParam_adc2_dac0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc2_dac0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc2_dac0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc2_dac1].label, "adc2_dac1");
  strcpy(gModuleData->paramDesc[eParam_adc2_dac1].unit, "u");
  gModuleData->paramDesc[eParam_adc2_dac1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc2_dac1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc2_dac1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc2_dac2].label, "adc2_dac2");
  strcpy(gModuleData->paramDesc[eParam_adc2_dac2].unit, "u");
  gModuleData->paramDesc[eParam_adc2_dac2].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc2_dac2].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc2_dac2].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc2_dac3].label, "adc2_dac3");
  strcpy(gModuleData->paramDesc[eParam_adc2_dac3].unit, "u");
  gModuleData->paramDesc[eParam_adc2_dac3].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc2_dac3].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc2_dac3].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc3_dac0].label, "adc3_dac0");
  strcpy(gModuleData->paramDesc[eParam_adc3_dac0].unit, "u");
  gModuleData->paramDesc[eParam_adc3_dac0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc3_dac0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc3_dac0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc3_dac1].label, "adc3_dac1");
  strcpy(gModuleData->paramDesc[eParam_adc3_dac1].unit, "u");
  gModuleData->paramDesc[eParam_adc3_dac1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc3_dac1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc3_dac1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc3_dac2].label, "adc3_dac2");
  strcpy(gModuleData->paramDesc[eParam_adc3_dac2].unit, "u");
  gModuleData->paramDesc[eParam_adc3_dac2].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc3_dac2].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc3_dac2].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_adc3_dac3].label, "adc3_dac3");
  strcpy(gModuleData->paramDesc[eParam_adc3_dac3].unit, "u");
  gModuleData->paramDesc[eParam_adc3_dac3].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_adc3_dac3].min = 0x00000000; 
  gModuleData->paramDesc[eParam_adc3_dac3].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del0_dac0].label, "del0_dac0");
  strcpy(gModuleData->paramDesc[eParam_del0_dac0].unit, "u");
  gModuleData->paramDesc[eParam_del0_dac0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del0_dac0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del0_dac0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del0_dac1].label, "del0_dac1");
  strcpy(gModuleData->paramDesc[eParam_del0_dac1].unit, "u");
  gModuleData->paramDesc[eParam_del0_dac1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del0_dac1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del0_dac1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del0_dac2].label, "del0_dac2");
  strcpy(gModuleData->paramDesc[eParam_del0_dac2].unit, "u");
  gModuleData->paramDesc[eParam_del0_dac2].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del0_dac2].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del0_dac2].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del0_dac3].label, "del0_dac3");
  strcpy(gModuleData->paramDesc[eParam_del0_dac3].unit, "u");
  gModuleData->paramDesc[eParam_del0_dac3].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del0_dac3].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del0_dac3].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del1_dac0].label, "del1_dac0");
  strcpy(gModuleData->paramDesc[eParam_del1_dac0].unit, "u");
  gModuleData->paramDesc[eParam_del1_dac0].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del1_dac0].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del1_dac0].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del1_dac1].label, "del1_dac1");
  strcpy(gModuleData->paramDesc[eParam_del1_dac1].unit, "u");
  gModuleData->paramDesc[eParam_del1_dac1].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del1_dac1].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del1_dac1].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del1_dac2].label, "del1_dac2");
  strcpy(gModuleData->paramDesc[eParam_del1_dac2].unit, "u");
  gModuleData->paramDesc[eParam_del1_dac2].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del1_dac2].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del1_dac2].max = FR32_MAX;

  strcpy(gModuleData->paramDesc[eParam_del1_dac3].label, "del1_dac3");
  strcpy(gModuleData->paramDesc[eParam_del1_dac3].unit, "u");
  gModuleData->paramDesc[eParam_del1_dac3].type = PARAM_TYPE_FRACT; 
  gModuleData->paramDesc[eParam_del1_dac3].min = 0x00000000; 
  gModuleData->paramDesc[eParam_del1_dac3].max = FR32_MAX; 
}
// EOF
