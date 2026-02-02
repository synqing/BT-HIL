/*
 * Emotiscope-parity tempo: 1024 novelty history, 96-bin Goertzel bank (48–144 BPM).
 * Authority: v1.1_build/tempo.h, global_defines.h.
 */
#ifndef P4_AUDIO_PRODUCER_TEMPO_EMOTISCOPE_H
#define P4_AUDIO_PRODUCER_TEMPO_EMOTISCOPE_H

#ifdef __cplusplus
extern "C" {
#endif

#define P4_NOVELTY_HISTORY_LENGTH 1024
#define P4_NOVELTY_LOG_HZ         50.0f    /* Emotiscope parity: novelty feed at 50 Hz */
#define P4_NUM_TEMPI              96
#define P4_TEMPO_LOW              48
#define P4_TEMPO_HIGH             (P4_TEMPO_LOW + P4_NUM_TEMPI)

/* Call once at startup (after audio producer start or before first feed). */
void tempo_emotiscope_init(void);

/*
 * Feed one novelty sample (spectral flux) and optional VU positive delta.
 * Then normalize, compute 96 tempo magnitudes, smooth, and update output.
 * flux: spectral flux (e.g. log1p(flux) is logged).
 * vu_positive_delta: optional, 0 if not available.
 */
void tempo_emotiscope_feed(float flux, float vu_positive_delta);

/* Output: BPM (48–144), confidence (0–1), beat phase 0–1. */
void tempo_emotiscope_get(float *bpm_out, float *conf_out, float *beat_phase_0_1_out);

#ifdef __cplusplus
}
#endif

#endif /* P4_AUDIO_PRODUCER_TEMPO_EMOTISCOPE_H */
