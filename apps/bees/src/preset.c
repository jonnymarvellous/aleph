/* preset.c
 * bees
 * aleph
 *
 * definitions for preset management 
 *

 TODO: operator network hashes

 */

//#include <stdio.h>
// asf
#ifdef ARCH_AVR32
#include "print_funcs.h"
#endif
// bees
#include "net_protected.h"
#include "param.h"
#include "preset.h"
// aleph
#include "memory.h"
#include "simple_string.h"
#include "types.h"


//=================================
//====== variables

/// array of presets
preset_t* presets;

//=================================
//====== function definitions

// initialize
void presets_init(void) {
  u32 i, j;

  presets = (preset_t*)alloc_mem(NET_PRESETS_MAX * sizeof(preset_t));
  
  for(i=0; i<NET_PRESETS_MAX; i++) {

    // empty name
    for(j=0; j<PRESET_NAME_LEN; j++) {
      presets[i].name[j] = '\0';
    }
    str_copy(presets[i].name, "_", PRESET_NAME_LEN);

    // empty inputs
    for(j=0; i<NET_INS_MAX; i++) {
      presets[i].ins[j].idx = -1;
      presets[i].ins[j].value = 0;
      presets[i].ins[j].enabled = 0;
    }
    // empty outputs
    for(j=0; i<NET_OUTS_MAX; i++) {
      presets[i].outs[j].target = -1;
      presets[i].outs[j].outIdx = -1;
      presets[i].outs[j].enabled = 0;
    }

    // empty params
    for(j=0; i<NET_PARAMS_MAX; i++) {
      presets[i].params[j].idx = -1;
      presets[i].params[j].value = 0;
      presets[i].params[j].enabled = 0;
    } 
  }
  
  // 

}

// de-initialize
void presets_deinit(void) {
}

// store a particular input
void preset_store_in(u32 preIdx, u32 inIdx) {  
  presets[preIdx].ins[inIdx].enabled = net_get_in_preset(inIdx);
  presets[preIdx].ins[inIdx].value = net_get_in_value(inIdx);
}

// store a particular output
void preset_store_out(u32 preIdx, u32 outIdx) {
  presets[preIdx].outs[outIdx].enabled = net_get_out_preset(outIdx);
  presets[preIdx].outs[outIdx].target = net_get_target(outIdx);  
}

// store everything enabled in given preset
void preset_store(u32 preIdx) {
  u16 i;
  // ins
  for(i=0; i<NET_INS_MAX; i++) {
    if( net_get_in_preset(i) ) {
      presets[preIdx].ins[i].value = net_get_in_value(i);
      presets[preIdx].ins[i].enabled = 1;
    }
  }
  // outs
  for(i=0; i<NET_OUTS_MAX; i++) {
    if(net_get_out_preset(i)) {
      presets[preIdx].outs[i].target = net_get_target(i);
      presets[preIdx].outs[i].enabled = 1;
    }
  }
  // params
  for(i=0; i<NET_PARAMS_MAX; i++) {
    if(get_param_preset(i)) {
      presets[preIdx].params[i].value =  get_param_value( i );
      presets[preIdx].params[i].enabled = 1;
    }
  }
}

// recall everything enabled in given preset
void preset_recall(u32 preIdx) {
  u16 i;
  // ins
  for(i=0; i<NET_INS_MAX; i++) {
    if(presets[preIdx].ins[i].enabled) {
      net_set_in_value( i, presets[preIdx].ins[i].value );
    }
  }
  // outs
  for(i=0; i<NET_OUTS_MAX; i++) {
    if(presets[preIdx].outs[i].enabled) {
      net_connect( i, presets[preIdx].outs[i].target );
    }
  }
  // params
  for(i=0; i<NET_PARAMS_MAX; i++) {
    if(presets[preIdx].params[i].enabled) {
      set_param_value( i, presets[preIdx].params[i].value );
    }
  }
}

// preset name
char* preset_name(u32 id) {
  //  print_dbg("\r\n request for preset name at address: ");
  //  print_dbg_hex((u32)&(presets[id]));
  return presets[id].name;
}
