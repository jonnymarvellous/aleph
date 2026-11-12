/* params.c
   wash - parameter descriptors.
   compiled on the host (make desc / make deploy) to write wash.dsc.
*/

#include <stdio.h>
#include <string.h>
#include "module.h"
#include "param_common.h"
#include "params.h"

static void set_desc(ParamDesc* d, const char* label, ParamType type,
                     s32 min, s32 max, u8 radix) {
  strncpy(d->label, label, PARAM_LABEL_LEN - 1);
  d->label[PARAM_LABEL_LEN - 1] = '\0';
  d->type = type;
  d->min = min;
  d->max = max;
  d->radix = radix;
}

static void set_amp(ParamDesc* d, const char* label) {
  set_desc(d, label, eParamTypeAmp, 0, FR32_MAX, 1);
}

static void set_bool(ParamDesc* d, const char* label) {
  set_desc(d, label, eParamTypeBool, 0, 1, 2);
}

void fill_param_desc(ParamDesc* desc) {
  char label[PARAM_LABEL_LEN];
  int t, k, src;

  set_amp(&desc[eParamIn0], "in1_gain");
  set_amp(&desc[eParamIn1], "in2_gain");
  set_amp(&desc[eParamIn2], "in3_gain");
  set_amp(&desc[eParamIn3], "in4_gain");
  set_amp(&desc[eParamDry], "dry");

  for(t = 0; t < WASH_NUM_TRACKS; t++) {
    // labels use 1-based track numbers, to match the front panel
    snprintf(label, sizeof(label), "t%d_from_in", t + 1);
    set_amp(&desc[TRACK_PARAM(t, eTrkFromIn)], label);

    // the other three tracks, ascending
    k = 0;
    for(src = 0; src < WASH_NUM_TRACKS; src++) {
      if(src == t) { continue; }
      snprintf(label, sizeof(label), "t%d_from_t%d", t + 1, src + 1);
      set_amp(&desc[TRACK_PARAM(t, eTrkFromA + k)], label);
      k++;
    }

    snprintf(label, sizeof(label), "t%d_feedback", t + 1);
    set_amp(&desc[TRACK_PARAM(t, eTrkFb)], label);

    snprintf(label, sizeof(label), "t%d_level", t + 1);
    set_amp(&desc[TRACK_PARAM(t, eTrkLevel)], label);

    snprintf(label, sizeof(label), "t%d_pan", t + 1);
    set_amp(&desc[TRACK_PARAM(t, eTrkPan)], label);

    snprintf(label, sizeof(label), "t%d_mute", t + 1);
    set_bool(&desc[TRACK_PARAM(t, eTrkMute)], label);

    snprintf(label, sizeof(label), "t%d_rec", t + 1);
    set_bool(&desc[TRACK_PARAM(t, eTrkRec)], label);

    snprintf(label, sizeof(label), "t%d_ovr", t + 1);
    set_bool(&desc[TRACK_PARAM(t, eTrkOvr)], label);

    snprintf(label, sizeof(label), "t%d_once", t + 1);
    set_bool(&desc[TRACK_PARAM(t, eTrkOnce)], label);

    snprintf(label, sizeof(label), "t%d_play", t + 1);
    set_bool(&desc[TRACK_PARAM(t, eTrkPlay)], label);

    snprintf(label, sizeof(label), "t%d_clear", t + 1);
    set_bool(&desc[TRACK_PARAM(t, eTrkClear)], label);

    // read-only
    snprintf(label, sizeof(label), "t%d_pos", t + 1);
    set_desc(&desc[TRACK_PARAM(t, eTrkPos)], label, eParamTypeFix, 0, FR32_MAX, 32);

    snprintf(label, sizeof(label), "t%d_len", t + 1);
    set_desc(&desc[TRACK_PARAM(t, eTrkLen)], label, eParamTypeFix, 0, FR32_MAX, 32);

    snprintf(label, sizeof(label), "t%d_state", t + 1);
    set_desc(&desc[TRACK_PARAM(t, eTrkState)], label, eParamTypeShort, 0, 31, 16);
  }
}
