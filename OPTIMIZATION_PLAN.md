# grblHAL-sim performance plan (x64, code-only, no functional change)

Goal: make the simulator more responsive and able to hold real-time (and faster)
pace, like a real MCU that never lags its own clock. Functionality and
simulation accuracy (exact tick at which every IRQ fires) must be preserved.

## Measured baseline (2026-07-10, WSL2 x64)

Workload: `printf '$X\nG1X10F600\nG1X0\n\x06'` piped to `grblHAL_sim -t 0 -n -g /dev/null`
(~3.1 s of simulated time: 1 s serial-input warmup + streaming + 2 s of motion).

- Current build has **no `CMAKE_BUILD_TYPE`** → compiled at **-O0**. Wall: **1.55 s**, CPU: 2.8 s (both threads pegged).
- Same code at `-O3` (`-DCMAKE_BUILD_TYPE=Release`): Wall: **0.87 s** → **1.8× end-to-end, zero code change**.
- gprof of the `-O2` build (sim thread dominates samples; grbl thread visible via call counts):

| % time | function | calls | note |
|---|---|---|---|
| 31% | `mcu_master_clock` | 23.7 M | once per simulated CPU tick |
| 6.4% | `sim_loop` | — | incl. per-tick float division for `sim_time` |
| 5.9% + 2.5% | `print_steps` + `grbl_per_tick` | 23.7 M | called every tick even when step reporting is off |
| ~3% | `plan_get_current_block` | 29.7 M | 24 M of these from `print_steps` |
| ~25% | `protocol_exec_rt_system`, `task_execute`, `serialGetC`, `st_prep_buffer`, … | ~5 M each | grbl thread busy-spinning its main loop |

Key structural fact: `sim_loop` does one full loop iteration per simulated tick —
16 M iterations per simulated second — even when the next interesting event
(timer expiry, serial byte) is thousands of ticks away.

## Ordered plan (biggest win first)

### 1. Build at -O3 by default (trivial, measured 1.8×; -O2 measured ~20% slower)
`CMakeLists.txt`: default `CMAKE_BUILD_TYPE` to `Release` when unset (keep it
overridable). Add `-fno-strict-aliasing` (negligible cost; removes a UB class
common in embedded-style code).

Safety notes (verified 2026-07-10):
- The real optimization hazard in this codebase is not -O3-specific: it is
  cross-thread sharing of plain variables (embedded ISR idioms running on OS
  threads). Those are equally exposed at -O1/-O2, so lowering the level buys
  no safety — fix the variables instead (see item #4). Empirically checked:
  the -O3 binary still re-reads `delay.ms` from memory in the
  `driver_delay_ms` spin loop (disassembly verified) and a `G4 P1` dwell
  completes; grbl core stream ring-buffer head/tail are already `volatile`,
  which is sound on x86-64 TSO for SPSC use. Still land the `volatile` fixes
  in item #4 in the same PR as this flag change — today's behavior is
  compiler luck, not a contract.
- **Defer LTO** until item #4's shared-state audit is done: several racy
  variables (e.g. `ticks` read via the `hal.get_elapsed_ticks` pointer) are
  currently protected only by cross-TU call boundaries, which LTO dissolves.
- Skip `-march=native`, or pair it with `-ffp-contract=off`: GCC's default
  FMA contraction would change planner float rounding and break the
  byte-identical output diffs used as the regression harness. Plain -O3 does
  not alter FP semantics (no -ffast-math).

### 2. Event-driven clock: batch-advance `masterclock` to the next event (order-of-magnitude)
Replace per-tick iteration in `sim_loop`/`mcu_master_clock` with:
compute `next_event = min(` ticks-to-expiry of each enabled timer (incl.
prescaler), systick expiry, `next_byte_tick`, `target_ticks` `)`, jump
`masterclock` forward in one step, then run the event's ISR **at exactly the
same tick value it fires today**. Recompute after every event (ISRs reprogram
timers).

- This is the change that makes it behave like a real MCU: today the sim tops
  out around a few × real time and can *fall behind* real time under load
  (jitter, laggy status/jog response). Event skipping removes the per-tick tax
  entirely; idle/waiting periods become free.
- Care needed (accuracy contract):
  - timer reload semantics in `mcu.c` (`value==0 → reload without IRQ`, IRQ on 1→0),
    prescaler behavior, `enable`/`irq_enable` flags;
  - GPIO pin-change IRQs are raised outside the clock (from `on_byte` /
    `mcu_gpio_in`) and serviced on the *next* tick — after any GPIO input
    change, the next event must be "now + 1 tick";
  - `sim.on_tick` (→ `print_steps`) only does work when a block changes or
    `sim_time` crosses `next_print_time`; make the print threshold an event so
    `-r` output timestamps stay identical.
- **Verification (required):** instrument old and new builds to log
  `(masterclock, irq_number)` for every ISR dispatch; run several g-code
  workloads (incl. homing-style limit toggles and `-r 0.01` step reporting) and
  diff the traces byte-for-byte, plus diff serial/block/step output files.

### 3. Per-tick micro-fixes (low-risk; independent of #2, also its fallback — ~1.5–2× on sim thread)
- `simulator.c:67`: delete the per-tick `sim.sim_time = (float)masterclock/(float)F_CPU`
  (a u64→float convert + divide, 16 M/s). Compute lazily in `print_steps` from
  `sim.masterclock`. Keep the same float rounding if byte-identical `-r` output matters.
- `grbl_interface.c print_steps`: return on `next_print_time == 0.0` (the
  default) *before* calling `plan_get_current_block()`; keep the
  `exit_REQ`/`state_get()` check first. Removes ~24 M calls/s.
- `mcu.c mcu_master_clock`: keep a bitmask of enabled timers and a global
  `gpio_irq_pending` flag (set in `mcu_gpio_in`, cleared when serviced) so an
  idle tick tests two words instead of scanning 3 timers + 10 GPIO ports;
  replace the timer `switch` with `isr[Timer0_IRQ + i]()`.

### 4. Stop the grbl thread from burning a full host core
grbl's main loop and `driver_delay_ms`'s `while(delay.ms);` spin at host speed
(~5.3 M polls of `protocol_execute_realtime` in a 0.9 s run; process CPU ≈ 2×
wall). On real hardware that spin is authentic; on the host it steals cycles
from the sim thread and hurts interactive responsiveness.
- Conservative: `sched_yield()` in `sim_process_realtime` (`driver.c`) and in
  the `delay.ms` wait loop.
- Better: µs-scale adaptive sleep only when rx buffer empty, no delay pending,
  and state idle — but measure that high step rates (planner starvation) are
  unaffected before keeping it.
- While there: `delay.ms` is read in a spin loop but `delay` is not `volatile`
  — works today at -O3 by luck; make it `volatile` (needed anyway once builds
  default to -O3, item #1).

### 5. Serial polling syscall diet
`platform_linux.c platform_poll_stdin` does `tcgetattr` + 2× `tcsetattr` +
`select` + `getchar` per poll, once per simulated byte-time (~11.5 k/s × speedup).
Set the terminal raw **once** in `platform_init()`, restore in
`platform_terminate()` + `atexit`; poll with one non-blocking `read()`.
Same for `sim_socket_in` (`main.c`): put the accepted socket in `O_NONBLOCK`
and use plain `read()`; keep `select` only for detecting new connections.
Matters more as items 1–3 raise the achievable speedup ceiling.

### 6. Smaller / later (measure first)
- Shrink the 100 ms control frame in `sim_loop` once the tick loop is cheap —
  lowers worst-case interactive latency/jitter at `-t 1`.
- `fflush` per line in `print_steps` only matters with `-r`; leave unless profiled.

## Out of scope but observed
- `mcu.c mcu_gpio_in`: `changed`/`bitflag` are `uint8_t` while ports are 16-bit —
  pin-change IRQs can never fire for pins 8–15. Latent correctness bug (all
  current masks are 8-bit), not a perf item. Worth a separate fix/issue.

## Benchmark & regression harness for every item
- Speed: fixed `EEPROM.DAT`, the workload above, `time` at `-t 0`; also a long
  job (many short segments) to stress the stepper ISR rate.
- Correctness: byte-diff of serial output, block output, `-r 0.01` step trace,
  and (for #2) the ISR `(tick, irq)` trace.
