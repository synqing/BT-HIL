/*
 * Tempo stabilizer: octave resolver + hysteresis so published BPM doesn't flip
 * between half-time and double-time (e.g. 64.7 vs 125). Raw tempo from the
 * detector is turned into a stable tempo_out for visuals.
 */
#ifndef P4_AUDIO_PRODUCER_TEMPO_STABILIZER_H
#define P4_AUDIO_PRODUCER_TEMPO_STABILIZER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Valid BPM range for candidates */
#define TEMPO_STABILIZER_BPM_MIN 60.0f
#define TEMPO_STABILIZER_BPM_MAX 180.0f

/* Confidence below this: hold tempo_out (do not flip) */
#define TEMPO_STABILIZER_CONF_MIN 0.20f

/* Same candidate must win this many consecutive updates before switching */
#define TEMPO_STABILIZER_N_CONSECUTIVE 6

/**
 * Update stabilizer with new raw tempo and confidence (call every slow-lane tick).
 * Internally: build candidates (raw, 2*raw, 0.5*raw), pick closest to previous
 * tempo_out, apply hysteresis (N consecutive wins + conf >= CONF_MIN to switch).
 */
void tempo_stabilizer_update(float tempo_raw, float conf);

/**
 * Get current stabilized BPM to publish to frame. Returns 0 if never set.
 */
float tempo_stabilizer_get_bpm(void);

/**
 * Log one line at most once per second: tempo_raw, tempo_out, conf, chosen mode (1x/2x/0.5x).
 * Call from main loop / heartbeat.
 */
void tempo_stabilizer_log_if_due(void);

#ifdef __cplusplus
}
#endif

#endif /* P4_AUDIO_PRODUCER_TEMPO_STABILIZER_H */
