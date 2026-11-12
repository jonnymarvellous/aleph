/* cascade.c
   aleph-audio

   4-track cascading looper for live soundscapes.

   stage 1:
   - 4 mono loop tracks, 16-bit samples in SDRAM (~150 s each)
   - routing matrix into each track: live input + the 3 other tracks
   - soft limiter on each track input (feedback paths can't run away)
   - per-track: record / overdub (latched) / dub once / play-stop / clear,
     feedback, level, pan, mute
   - stereo mix (outs 1/2, duplicated on 3/4) with dry level

   signal flow per sample:
     live = sum(in[i] * inGain[i])
     for each track n:
       x = limit( live * g[n][in] + sum over other tracks m: out[m] * g[n][m] )
       (track outputs from the previous sample feed the matrix, so any
        routing, including feedback from later tracks, is well-defined)
       recording: buf[pos] = x
       playing:   y = buf[pos]; if writing: buf[pos] = limit(y*fb + x)
       out[n] = y * seam fade * play envelope
     mix = sum(out[n] * level/pan) + dry

   threading note:
   module_set_param() runs in the SPI interrupt, which the audio interrupt
   can pre-empt. so set_param only does single-word writes (slew targets,
   one-byte trigger flags); all state changes happen in the audio frame.
*/

#include <string.h>

#include "fix.h"
#include "types.h"

#include "bfin_core.h"
#include "fract_math.h"

#include "module.h"
#include "params.h"

//-----------------------------
//---- constants

#define NTRK CASCADE_NUM_TRACKS

// 150 seconds per track at 48 kHz, 16-bit.
// 4 tracks = 57.6 MB of the 64 MB SDRAM, leaving ~6 MB for effects later.
#define TRACK_FRAMES 7200000

// 5 ms linear fades (play/stop envelopes, write envelope, loop seam)
#define FADE_FRAMES 240
#define FADE_STEP (FR32_MAX / FADE_FRAMES)

// anything shorter than this when closing a first recording is discarded
#define MIN_LOOP_FRAMES (FADE_FRAMES * 4)

// one-pole parameter smoothing, ~5 ms time constant at 48 kHz
#define SLEW_COEFF ((fract32)(0.9958 * 2147483647.0))

// soft limiter: linear below 0.5, smooth curve up to a 0.75 ceiling
#define LIMIT_KNEE (FR32_MAX >> 1)

// source index of the live input in the matrix source array
#define SRC_LIVE NTRK

// actions that wait for the playback envelope to fade out first
enum pendingActions {
  ePendNone = 0,
  ePendStop,
  ePendClear,
  ePendRerecord
};

//-----------------------------
//---- types

typedef struct {
  fract32 x; // target
  fract32 y; // current
} slew_t;

typedef struct {
  volatile fract16* data;
  u32 len;        // loop length in frames (0 = empty; grows while recording)
  u32 pos;        // playhead / record head
  u8 state;       // enum trackStates
  u8 ovr;         // latched overdub
  u32 onceLeft;   // frames left in the current dub-once (0 = inactive)
  u8 pending;     // enum pendingActions
  fract32 outEnv; // playback envelope 0..1
  fract32 wrEnv;  // write envelope 0..1

  slew_t gain[4]; // matrix gains into this track: live, then other tracks
  slew_t fb;
  slew_t mixL;
  slew_t mixR;

  // raw values for computing the mix gains
  fract32 level;
  fract32 pan;
  u8 mute;

  fract32 out;    // output sample (read by the matrix on the next frame)

  // trigger flags, set by module_set_param, consumed in the audio frame
  volatile u8 reqRec;
  volatile u8 reqOvr;
  volatile u8 reqOnce;
  volatile u8 reqPlay;
  volatile u8 reqClear;
} track_t;

// external memory layout
typedef struct _cascadeData {
  ModuleData super;
  ParamData mParamData[eParamNumParams];
  volatile fract16 audioBuffer[NTRK][TRACK_FRAMES];
} cascadeData;

//-----------------------------
//---- variables

ModuleData* gModuleData;

static cascadeData* pData;
static track_t tracks[NTRK];

static slew_t inGain[4];
static slew_t dryGain;

// matrix source for each (track, slot): slot 0 is the live input,
// slots 1-3 are the other tracks in ascending order
static u8 srcMap[NTRK][4];

//-----------------------------
//---- helpers

static inline void slew_init(slew_t* s, fract32 v) {
  s->x = v;
  s->y = v;
}

static inline fract32 slew_next(slew_t* s) {
  s->y = add_fr1x32(s->x, mult_fr1x32x32(SLEW_COEFF, sub_fr1x32(s->y, s->x)));
  return s->y;
}

// identity below the knee (0.5); above it y = 0.5 + u - u^2 (u = |x| - 0.5),
// which meets the line smoothly and flattens out at 0.75 for full-scale input.
static inline fract32 soft_limit(fract32 x) {
  fract32 a = abs_fr1x32(x);
  fract32 u, y;
  if(a <= LIMIT_KNEE) {
    return x;
  }
  u = sub_fr1x32(a, LIMIT_KNEE);
  y = add_fr1x32(LIMIT_KNEE, sub_fr1x32(u, mult_fr1x32x32(u, u)));
  return (x < 0) ? negate_fr1x32(y) : y;
}

// linear ramp towards 0 or 1
static inline fract32 ramp(fract32 v, u8 up) {
  if(up) {
    return add_fr1x32(v, FADE_STEP);
  } else {
    v = sub_fr1x32(v, FADE_STEP);
    return (v < 0) ? 0 : v;
  }
}

// short fade in/out at the loop seam, so a first recording doesn't click
static inline fract32 seam_gain(u32 pos, u32 len) {
  u32 fromEnd;
  if(pos < FADE_FRAMES) {
    return (fract32)(pos * FADE_STEP);
  }
  fromEnd = len - 1 - pos;
  if(fromEnd < FADE_FRAMES) {
    return (fract32)(fromEnd * FADE_STEP);
  }
  return FR32_MAX;
}

// pan law: L = 1 - p^2, R = 1 - (1-p)^2  (close to equal power, cheap)
static void update_mix_gains(track_t* t) {
  fract32 p = t->pan;
  fract32 q = sub_fr1x32(FR32_MAX, p);
  fract32 l = t->mute ? 0 : t->level;
  t->mixL.x = mult_fr1x32x32(l, sub_fr1x32(FR32_MAX, mult_fr1x32x32(p, p)));
  t->mixR.x = mult_fr1x32x32(l, sub_fr1x32(FR32_MAX, mult_fr1x32x32(q, q)));
}

//-----------------------------
//---- track state machine

static void track_start_recording(track_t* t) {
  t->state = eTrackRecording;
  t->pos = 0;
  t->len = 0;
  t->ovr = 0;
  t->onceLeft = 0;
  t->outEnv = 0;
  t->wrEnv = 0;
}

static void track_close_loop(track_t* t) {
  if(t->pos < MIN_LOOP_FRAMES) {
    // too short to be a deliberate loop: discard
    t->state = eTrackEmpty;
    t->len = 0;
  } else {
    t->len = t->pos;
    t->state = eTrackPlaying;
  }
  t->pos = 0;
}

static void track_clear(track_t* t) {
  t->state = eTrackEmpty;
  t->len = 0;
  t->pos = 0;
  t->ovr = 0;
  t->onceLeft = 0;
  t->wrEnv = 0;
}

// consume trigger flags. called once per frame per track.
static void track_handle_requests(track_t* t) {
  u8 rec, ovr, once, play, clear;

  // finish an action that was waiting for the fade-out
  if(t->pending != ePendNone) {
    if(t->outEnv > 0) {
      return; // still fading; leave new triggers queued
    }
    switch(t->pending) {
    case ePendStop:
      t->state = eTrackStopped;
      t->onceLeft = 0;
      break;
    case ePendClear:
      track_clear(t);
      break;
    case ePendRerecord:
      track_start_recording(t);
      break;
    default:
      break;
    }
    t->pending = ePendNone;
  }

  rec = t->reqRec;
  ovr = t->reqOvr;
  once = t->reqOnce;
  play = t->reqPlay;
  clear = t->reqClear;
  if(!(rec | ovr | once | play | clear)) {
    return;
  }
  t->reqRec = t->reqOvr = t->reqOnce = t->reqPlay = t->reqClear = 0;

  switch(t->state) {
  case eTrackEmpty:
    if(rec || ovr || once) {
      track_start_recording(t);
    }
    break;

  case eTrackRecording:
    if(clear) {
      track_clear(t);
    } else if(rec || ovr || once) {
      track_close_loop(t);
    }
    break;

  case eTrackPlaying:
    if(clear) {
      t->pending = ePendClear;
    } else if(rec) {
      t->pending = ePendRerecord;
    } else if(play) {
      t->pending = ePendStop;
    } else {
      if(ovr) {
        t->ovr = !t->ovr;
      }
      if(once) {
        t->onceLeft = t->len;
      }
    }
    break;

  case eTrackStopped:
    if(clear) {
      track_clear(t);
    } else if(rec) {
      track_start_recording(t);
    } else if(play) {
      t->pos = 0;
      t->state = eTrackPlaying;
    } else if(ovr) {
      t->ovr = !t->ovr;
    }
    break;

  default:
    break;
  }
}

// process one sample for one track. x = limited matrix input.
static void track_process(track_t* t, fract32 x) {
  fract32 y, d, w, fbAmt;
  u8 audible, writing;

  track_handle_requests(t);

  fbAmt = slew_next(&(t->fb));

  switch(t->state) {

  case eTrackRecording:
    t->data[t->pos] = trunc_fr1x32(x);
    t->pos++;
    t->len = t->pos; // so the controller can see it growing
    if(t->pos >= TRACK_FRAMES) {
      track_close_loop(t);
    }
    t->out = 0;
    break;

  case eTrackPlaying:
    audible = (t->pending == ePendNone);
    writing = audible && (t->ovr || (t->onceLeft > 0));
    t->outEnv = ramp(t->outEnv, audible);
    t->wrEnv = ramp(t->wrEnv, writing);

    y = ((fract32)(t->data[t->pos])) << 16;

    w = t->wrEnv;
    if(w > 0) {
      // new = y - w * (y * (1 - fb) - x)
      // w = 1: y*fb + x.  w = 0: y unchanged.
      d = sub_fr1x32(x, mult_fr1x32x32(y, sub_fr1x32(FR32_MAX, fbAmt)));
      t->data[t->pos] = trunc_fr1x32(soft_limit(add_fr1x32(y, mult_fr1x32x32(w, d))));
    }

    t->out = mult_fr1x32x32(mult_fr1x32x32(y, seam_gain(t->pos, t->len)), t->outEnv);

    if(t->onceLeft > 0) {
      t->onceLeft--;
    }
    t->pos++;
    if(t->pos >= t->len) {
      t->pos = 0;
    }
    break;

  default: // empty, stopped
    t->outEnv = 0;
    t->wrEnv = 0;
    t->out = 0;
    break;
  }
}

static void report_track(int n) {
  track_t* t = &(tracks[n]);
  s32 st = t->state;
  if(t->ovr) { st |= TRACK_STATE_OVR_BIT; }
  if(t->onceLeft > 0) { st |= TRACK_STATE_ONCE_BIT; }
  if(t->mute) { st |= TRACK_STATE_MUTE_BIT; }
  gModuleData->paramData[TRACK_PARAM(n, eTrkPos)].value = (s32)t->pos;
  gModuleData->paramData[TRACK_PARAM(n, eTrkLen)].value = (s32)t->len;
  gModuleData->paramData[TRACK_PARAM(n, eTrkState)].value = st;
}

//-----------------------------
//---- module interface

static inline void param_setup(u32 id, ParamValue v) {
  gModuleData->paramData[id].value = v;
  module_set_param(id, v);
}

void module_init(void) {
  int n, k, src;

  pData = (cascadeData*)SDRAM_ADDRESS;
  gModuleData = &(pData->super);
  strcpy(gModuleData->name, "cascade");
  gModuleData->paramData = (ParamData*)pData->mParamData;
  gModuleData->numParams = eParamNumParams;

  // no need to clear the audio buffers: a track only ever plays back
  // the region it has already recorded.
  memset(tracks, 0, sizeof(tracks));
  for(n = 0; n < NTRK; n++) {
    tracks[n].data = pData->audioBuffer[n];
    tracks[n].state = eTrackEmpty;

    srcMap[n][0] = SRC_LIVE;
    k = 1;
    for(src = 0; src < NTRK; src++) {
      if(src != n) {
        srcMap[n][k++] = (u8)src;
      }
    }
  }

  for(k = 0; k < 4; k++) {
    slew_init(&(inGain[k]), 0);
  }
  slew_init(&dryGain, 0);

  // --- defaults
  // live bus: inputs 1+2 (a stereo source summed to mono)
  param_setup(eParamIn0, PARAM_AMP_6);
  param_setup(eParamIn1, PARAM_AMP_6);
  param_setup(eParamIn2, 0);
  param_setup(eParamIn3, 0);
  param_setup(eParamDry, PARAM_AMP_0);

  for(n = 0; n < NTRK; n++) {
    // matrix: live -> T1 at unity; a simple cascade T1 -> T2 -> T3 -> T4
    param_setup(TRACK_PARAM(n, eTrkFromIn), (n == 0) ? PARAM_AMP_0 : 0);
    for(k = 1; k < 4; k++) {
      param_setup(TRACK_PARAM(n, eTrkFromIn + k),
                  (srcMap[n][k] == n - 1) ? PARAM_AMP_3 : 0);
    }
    param_setup(TRACK_PARAM(n, eTrkFb), 0x73333333); // 0.9
    param_setup(TRACK_PARAM(n, eTrkLevel), PARAM_AMP_6);
    param_setup(TRACK_PARAM(n, eTrkMute), 0);
    // triggers and read-only values start at 0
    for(k = eTrkRec; k < eTrkNumFields; k++) {
      param_setup(TRACK_PARAM(n, k), 0);
    }
  }
  // spread the tracks a little across the stereo field
  param_setup(TRACK_PARAM(0, eTrkPan), 0x33333333); // 0.4
  param_setup(TRACK_PARAM(1, eTrkPan), 0x4ccccccc); // 0.6
  param_setup(TRACK_PARAM(2, eTrkPan), 0x26666666); // 0.3
  param_setup(TRACK_PARAM(3, eTrkPan), 0x59999999); // 0.7
}

void module_deinit(void) {
}

u32 module_get_num_params(void) {
  return eParamNumParams;
}

void module_process_frame(void) {
  fract32 src[NTRK + 1];
  fract32 live, x, l, r, o;
  int n, k;

  // live bus
  live = 0;
  for(k = 0; k < 4; k++) {
    live = add_fr1x32(live, mult_fr1x32x32(in[k], slew_next(&(inGain[k]))));
  }

  // matrix sources: previous track outputs + live input
  for(n = 0; n < NTRK; n++) {
    src[n] = tracks[n].out;
  }
  src[SRC_LIVE] = live;

  // tracks
  l = 0;
  r = 0;
  for(n = 0; n < NTRK; n++) {
    track_t* t = &(tracks[n]);
    x = 0;
    for(k = 0; k < 4; k++) {
      x = add_fr1x32(x, mult_fr1x32x32(src[srcMap[n][k]], slew_next(&(t->gain[k]))));
    }
    track_process(t, soft_limit(x));

    o = t->out;
    l = add_fr1x32(l, mult_fr1x32x32(o, slew_next(&(t->mixL))));
    r = add_fr1x32(r, mult_fr1x32x32(o, slew_next(&(t->mixR))));

    report_track(n);
  }

  // dry: inputs 1+3 -> left, 2+4 -> right
  o = slew_next(&dryGain);
  l = add_fr1x32(l, mult_fr1x32x32(add_fr1x32(in[0], in[2]), o));
  r = add_fr1x32(r, mult_fr1x32x32(add_fr1x32(in[1], in[3]), o));

  out[0] = l;
  out[1] = r;
  out[2] = l;
  out[3] = r;
}

void module_set_param(u32 idx, ParamValue v) {
  u32 n, field;
  track_t* t;

  if(idx < eParamTrackBase) {
    switch(idx) {
    case eParamIn0: inGain[0].x = v; break;
    case eParamIn1: inGain[1].x = v; break;
    case eParamIn2: inGain[2].x = v; break;
    case eParamIn3: inGain[3].x = v; break;
    case eParamDry: dryGain.x = v; break;
    default: break;
    }
    return;
  }

  idx -= eParamTrackBase;
  n = idx / eTrkNumFields;
  field = idx % eTrkNumFields;
  if(n >= NTRK) {
    return;
  }
  t = &(tracks[n]);

  switch(field) {
  case eTrkFromIn:
  case eTrkFromA:
  case eTrkFromB:
  case eTrkFromC:
    t->gain[field - eTrkFromIn].x = v;
    break;
  case eTrkFb:
    t->fb.x = v;
    break;
  case eTrkLevel:
    t->level = v;
    update_mix_gains(t);
    break;
  case eTrkPan:
    t->pan = v;
    update_mix_gains(t);
    break;
  case eTrkMute:
    t->mute = (v != 0);
    update_mix_gains(t);
    break;
  case eTrkRec:   if(v) { t->reqRec = 1; }   break;
  case eTrkOvr:   if(v) { t->reqOvr = 1; }   break;
  case eTrkOnce:  if(v) { t->reqOnce = 1; }  break;
  case eTrkPlay:  if(v) { t->reqPlay = 1; }  break;
  case eTrkClear: if(v) { t->reqClear = 1; } break;
  default:
    // read-only (pos / len / state): ignore
    break;
  }
}
