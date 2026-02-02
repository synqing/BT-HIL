# P4 Audio Producer — Gate Report

**Project:** p4_audio_producer (ESP32-P4 two-lane audio producer)  
**Authority:** [DEPARTMENTAL_AUDIO_PRODUCER_SPEC.md](DEPARTMENTAL_AUDIO_PRODUCER_SPEC.md)  
**Report date:** (fill when gates are run)

---

## Result (PASS/FAIL per gate)

| Gate | Result | Notes |
|------|--------|--------|
| Dept 0: Build + compile-time assertion + boot banner | (PASS/FAIL) | |
| Dept 1: Heartbeat runs on P4 | (PASS/FAIL) | |
| Dept 2: 0 capture overruns (10 min); fast lane within 8 ms; max margin logged | (PASS/FAIL) | |
| Dept 3: E2e latency ≤ 16 ms typical, ≤ 24 ms gate | (PASS/FAIL) | |
| Dept 4: HAL integrated; 10-min gate still pass | (PASS/FAIL) | |
| 72h stress: zero crashes; leak within target; 0 overruns; latency within gate | (PASS/FAIL) | |
| Dept 5: WiFi up; 10-min gate still pass | (PASS/FAIL) | |

---

## Logs (excerpts)

**Boot banner:**
```
( paste: P4 Audio Producer | Fs=16000 Hz Hop=128 samples (8 ms) | Spec 1.1 )
```

**Heartbeat + counters (example):**
```
( paste: P4 heartbeat N | capture_overruns=0 fast_overruns=0 seq=... max_fast_us=... )
```

**Latency (example):**
```
( paste: latency | audio max=... avg=... p99=... us | e2e max=... avg=... p99=... us | ... )
```

---

## Counters (final values)

- **10-minute run:** capture_overruns = ..., fast_lane_overruns = ..., published_seq = ..., max_fast_lane_us = ...
- **72h run (if applicable):** capture_overruns = ..., fast_lane_overruns = ..., published_seq = ...

---

## Latency (typical / gate)

- **Audio latency (capture → publish):** typical = ... ms, p99 = ... ms, max = ... ms
- **End-to-end (capture → visual commit):** typical = ... ms, gate (≤24 ms) = PASS/FAIL

---

## Leak (72h)

- **Heap at start:** ... bytes
- **Heap at end:** ... bytes
- **Leak target:** (e.g. zero growth or max N bytes/hour per project spec)
- **Result:** PASS/FAIL

---

## Commit hashes (per department)

| Department | Commit hash (that passed gate) |
|------------|--------------------------------|
| Dept 0 | (fill) |
| Dept 1 | (fill) |
| Dept 2 | (fill) |
| Dept 3 | (fill) |
| Dept 4 | (fill) |
| Dept 5 | (fill) |

---

**Instructions:** Run 10-minute and (optionally) 72h stress tests; fill this report; paste log excerpts and commit hashes. Do not advance to Dept 5 WiFi bring-up until 72h stress passes.
