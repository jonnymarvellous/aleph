/* params.h
   wash - 4-track cascading looper

   parameter layout:
     global params, then 4 identical blocks of per-track params.
     per-track param index = eParamTrackBase + (track * eTrkNumFields) + field

   control params (rec / ovr / once / play / clear) are triggers:
   any non-zero value is a "press". zero is ignored, so a BEES key that
   sends 1 on press and 0 on release works directly.

   read-only params (pos / len / state) are written by the DSP so the
   controller can poll them (MSG_GET_PARAM_COM). setting them does nothing.
*/

#ifndef _ALEPH_WASH_PARAMS_H_
#define _ALEPH_WASH_PARAMS_H_

#include "param_common.h"

#define WASH_NUM_TRACKS 4

// amplitude constants
#define PARAM_AMP_0  (FR32_MAX)        // 0 dB
#define PARAM_AMP_3  (0x5a9df7ab)      // ~ -3 dB
#define PARAM_AMP_6  (FR32_MAX >> 1)   // ~ -6 dB
#define PARAM_AMP_12 (FR32_MAX >> 2)   // ~ -12 dB

// per-track fields
enum trackFields {
  // routing matrix "into" this track.
  // sources: live input, then the other three tracks in ascending order.
  // e.g. for track 2: in, t1, t3, t4
  eTrkFromIn,
  eTrkFromA,
  eTrkFromB,
  eTrkFromC,

  eTrkFb,      // feedback: how much old material survives while writing
  eTrkLevel,   // output level to the stereo mix
  eTrkPan,     // 0 = left, 0.5 = centre, 1 = right
  eTrkMute,    // bool: mutes the mix output only (matrix sends keep flowing)

  // triggers
  eTrkRec,     // empty: start recording. recording: close loop. playing: re-record
  eTrkOvr,     // toggle latched overdub (empty: start recording / recording: close)
  eTrkOnce,    // dub for exactly one loop cycle (empty: start / recording: close)
  eTrkPlay,    // toggle play / stop (restart is from the top)
  eTrkClear,   // erase the loop

  // read-only, written by the DSP
  eTrkPos,     // playhead position in samples
  eTrkLen,     // loop length in samples (0 = empty, grows while recording)
  eTrkState,   // bits 0-1: 0 empty, 1 recording, 2 playing, 3 stopped
               // bit 2: overdub on, bit 3: dub-once active, bit 4: muted

  eTrkNumFields
};

// track states (eTrkState bits 0-1)
enum trackStates {
  eTrackEmpty = 0,
  eTrackRecording = 1,
  eTrackPlaying = 2,
  eTrackStopped = 3
};

#define TRACK_STATE_OVR_BIT   (1 << 2)
#define TRACK_STATE_ONCE_BIT  (1 << 3)
#define TRACK_STATE_MUTE_BIT  (1 << 4)

// global params
enum params {
  // input gains into the live bus (mono sum of the 4 inputs)
  eParamIn0,
  eParamIn1,
  eParamIn2,
  eParamIn3,
  // dry level: inputs 1+3 -> left, 2+4 -> right
  eParamDry,

  eParamTrackBase,
  eParamNumParams = eParamTrackBase + (WASH_NUM_TRACKS * eTrkNumFields)
};

#define TRACK_PARAM(track, field) (eParamTrackBase + ((track) * eTrkNumFields) + (field))

extern void fill_param_desc(ParamDesc* desc);

#endif // header guard
