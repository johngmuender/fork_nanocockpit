# NanoCockpit code analysis

Read-only review of this tree (John's fork, `johngmuender/fork_nanocockpit`) against the paper *NanoCockpit: Performance-optimized Application Framework for AI-based Autonomous Nanorobotics*, Cereda, Giusti, Palossi, arXiv:2601.07476v2 (22 Apr 2026). No application source was modified for this note. The only new file is this document.

How to read the labels:

- **Code.** A fact that is in this repository, with a path and line.
- **Derived.** A conclusion obtained by reading that code, by arithmetic on its constants, or by checking a public header or release note outside the tree. Not a measurement on a drone.
- **Opinion.** A judgment. Not something the repository proves.

Build attempts are in [section 11](#11-what-was-actually-built-in-this-environment). Anything not listed there was not compiled, flashed, or run.

The paper's own supplementary Table III says the open-source drop contains the framework, small examples, and **one** full application (PULP-Frontnet). That matches this tree. The drone-to-drone and racing experiments are described in the paper and are not implemented here.

---

## 1. Repository layout

There is no `.gitmodules`. Nothing in this checkout is a git submodule. The Crazyflie firmware, the Crazyflie Python client, and the ESP-IDF `mdns` component are vendored copies. **Code.**

Top level:

| Path | Role |
| --- | --- |
| `README.md` | Build and flash sketch for the PULP-Frontnet demo. Pins Ubuntu 22.04 and Python 3.9. |
| `requirements.txt` | Host Python pins, including `cflib==0.1.20.1` and `numpy==1.22.4`. |
| `LICENSE` | Apache-2.0 for the NanoCockpit code. The vendored Crazyflie firmware is GPL/LGPL; see its own `LICENSE.txt`. |
| `docs/literature_review/` | Scopus query and the annotated CSV behind Figure 1 / Appendix A. Not executable. |
| `src/gap/` | GAP8 fabric-controller framework (`lib/`) and examples (`examples/`). |
| `src/stm32/app/` | PULP-Frontnet closed-loop controller, built as a Crazyflie app-layer image. |
| `src/stm32/crazyflie-firmware/` | Vendored Bitcraze STM32 firmware, with IDSIA edits (AI-deck UART, state snapshot, LED demo leftovers). |
| `src/nina/` | ESP32 (NINA-W10) CPX router: SPI slave in, Wi-Fi out. |
| `src/client/aideck_cpx_streamer/` | Host CPX client, Matplotlib viewer, ROS 2 viewer. |
| `src/client/aideck_cpx_msgs/` | ROS message package. |
| `src/client/crazyflie-clients-python/` | Vendored `cfclient` / `cfloader`. `setup.py` requires `cflib~=0.1.20.1`. |
| `src/docker/gapsdk/`, `src/docker/gap-riscv-gnu-toolchain/`, `src/docker/espidf/` | Dockerfiles for the two cross toolchains. |
| `tools/trace/esp32_trace.ipynb` | Notebook for ESP32 trace dumps. |
| `tools/wireshark/*.lua` | CPX dissectors for TCP, UDP, and the streamer. |

**Code.** `src/gap/lib/` is the framework: coroutines, camera, cluster open, CPX/SPI, UART protocol, streamer, queues, tracing. `src/gap/examples/` holds `coroutine`, `queue`, `queue_async`, `cpx`, `streamer`, and `pulp-frontnet`. **Code.** `src/gap/README.md` lines 5–6.

History on this fork is flat: `1d14874` is "Initial commit", then about 35 local commits (ESP-IDF 5.3.3, CPX bidirectional comments, README/GVSOC notes, setpoint priority, literature-review refresh). There is no upstream submodule SHA to diff against `idsia-robotics/nanocockpit`. **Code** (git log). **Opinion:** treat the tree as the artifact, not as a clean fork of a tagged upstream commit.

---

## 2. Build systems and toolchains

### 2.1 GAP8: GAP SDK, not pulp-sdk

**Code.** The supported SDK is GAP SDK **v3.8.1** (`gap_sdk` tag `release-v3.8.1`). `README.md` lines 52–59, `src/gap/README.md` lines 7–11, `src/docker/gapsdk/README.md` lines 6–7 and 79. The runtime the C files include is PMSIS (`#include <pmsis.h>`), which is the GAP SDK HAL. The older pulp-sdk is not referenced.

**Code.** The Docker image is Ubuntu 22.04 plus a separate toolchain image. The documented build arguments are `GAP_SDK_VERSION=3.8.1` and `GAP_TOOLCHAIN_VERSION=24.02` (`src/docker/gapsdk/README.md` lines 27–34). `src/docker/gapsdk/Dockerfile` checks out `tags/release-v${GAP_SDK_VERSION}` (lines 67–73) and applies local patches under `src/docker/gapsdk/patches/gapsdk/` for Ubuntu 22.04 and arm64 (lines 75–80).

**Code.** Both Dockerfiles clone over SSH (`git@github.com:...`) with BuildKit `--mount=type=ssh` (`src/docker/gapsdk/Dockerfile` lines 61–64, `src/docker/gap-riscv-gnu-toolchain/Dockerfile` lines 37–47). A machine without a GitHub SSH key cannot build the image from this Dockerfile. Prebuilt images are documented on GitLab (`registry.gitlab.com/eliacereda/gapsdk`), not built in this review.

**Code.** AutoTiler is not invoked. NNTool is not built: the Dockerfile comments out the TensorFlow 1.15 / `nntool` install (`src/docker/gapsdk/Dockerfile` lines 93–97). The network that ships is already-generated C. Layer files name the SDK in a generator banner, for example `src/gap/examples/pulp-frontnet/app/networks/frontnet-160x32-bgaug/src/layer0_BNReluConvolution.c` line 23 (`sdk gap_sdk`). The orchestration in `network.c` (directional L2 allocator, per-layer `pi_cl_team_fork`, L3 weight files) is the DORY style the paper describes in Appendix C. **Derived:** DORY and QuantLib are not in this repo; only their output is.

The app is a PMSIS makefile, not a custom build system:

```20:41:src/gap/examples/pulp-frontnet/Makefile
APP = pulp_frontnet
...
include app/app.mk
...
include $(RULES_DIR)/pmsis_rules.mk
```

`RULES_DIR` comes from the sourced `ai_deck` config inside the container (`src/docker/gapsdk/README.md` lines 58–62). Without that environment, `make` cannot start. **Derived.**

**Code.** GVSOC is documented (`src/gap/README.md` lines 27–41, root `README.md` does not repeat the flag). The symbol those instructions tell you to enable, `NETWORK_TEST_INPUT`, occurs only as a comment in `src/gap/examples/pulp-frontnet/config.h` line 53. No `.c` file reads it. The GVSOC camera/UART makefile hooks are commented out (`src/gap/examples/pulp-frontnet/Makefile` lines 37–39). A commit on this fork is titled "Disable gvsoc in pulp-frontnet example for now". **Opinion:** the GVSOC paragraph in `src/gap/README.md` is stale relative to the sources.

### 2.2 STM32: Crazyflie firmware 2021.06, app layer, not a separate OOT binary

**Code.** `src/stm32/README.md` lines 9–14 say the tree is "based on version 2021.06" and that the NRF51 is left on stock 2021.06. The vendored tree has no `VERSION` file and no `.git`, so the tag is the README's claim, not a git tag inside `crazyflie-firmware/`.

**Derived, checked against the public tag.** `platform_cf2.c` at Bitcraze tag `2021.06` has the same `CF20` and `CF21` entries. This tree comments out the Bolt entries (`RZ10`, `CB10`) that are active in that tag (`src/stm32/crazyflie-firmware/src/platform/platform_cf2.c` lines 36–64). So this is 2021.06 plus local edits, not a pristine tarball. A full file-by-file diff against the tag was not done.

**Code.** Platforms the makefile knows: `cf2` and `tag` only (`src/stm32/crazyflie-firmware/tools/make/platforms/cf2.mk`, `tag.mk`). Default `PLATFORM ?= cf2` (`src/stm32/crazyflie-firmware/Makefile` line 26). There is no `cf21bl` platform.

**Code.** The app is the 2021-era in-firmware app layer, not the later separately flashed app image. `src/stm32/app/Makefile` sets `APP=1`, adds the Frontnet objects, and `include`s the firmware makefile (lines 22–34). `APP=1` only adds `-DAPP_ENABLED=1` (`src/stm32/crazyflie-firmware/Makefile` lines 119–121). `system.c` then calls `appInit()` (`src/stm32/crazyflie-firmware/src/modules/src/system.c` lines 140–141). Frontnet defines `appInit` (`src/stm32/app/src/frontnet_main.c` lines 394–422), which overrides the weak one in `app_handler.c` (lines 51–58). `make cload` runs `python3 -m cfloader ... flash ... stm32-fw` (`src/stm32/crazyflie-firmware/Makefile` lines 21–25 and 441–443). One image is built and bootloaded. **Derived:** you cannot flash "just the app" onto a stock 2025 firmware; the controller is linked into this firmware.

**Code.** AI-deck is forced on: `CFLAGS += -DDECK_FORCE=bcAI` (`src/stm32/app/Makefile` line 25). The deck driver is `src/stm32/crazyflie-firmware/src/deck/drivers/src/aideck.c` (vid `0xBC`, pid `0x12`, name `bcAI`, lines 203–206).

**Code.** IDSIA edits inside the firmware, marked in comments: a latest-state queue in `stabilizer.c` (lines 179–197), flow-measurement hooks in `mm_flow.c`, LED changes tagged "IDSIA TII SUMMIT DEMO" in `ledseq.c` / `led.h`, and the AI-deck UART protocol. **Opinion:** the LED comments are leftover from a demo and are unrelated to the paper.

Bitcraze "toolbelt" is mentioned only in the vendored firmware README as a way to build HTML docs. This project does not use it. **Code.**

### 2.3 ESP32: ESP-IDF version does not match the root README

**Code.** `src/nina/dependencies.lock` line 15 pins `idf` **5.3.3**. `src/nina/main/idf_component.yml` allows `idf >= 4.4.8` and `espressif/mdns ~1.4` (resolved to 1.4.0 in the lock file). `src/docker/espidf/Dockerfile` line 1 and `src/docker/espidf/README.md` document **5.3.3**. `src/nina/README.md` line 12 says `esp 5.3.3`.

**Code.** The root `README.md` lines 71–77 still say **ESP-IDF v5.3.1** and `esp 5.3.1`. **Opinion:** follow the lock file and `src/nina/README.md`, not the root README. 5.3.1 vs 5.3.3 is a real footgun because `idf.py` will not match the committed `sdkconfig` if the container version differs.

**Code.** `sdkconfig` is committed and selects 2 MB flash (`CONFIG_ESPTOOLPY_FLASHSIZE_2MB`), station mode (AP off), SSID `drone_wifi`, password `wifi-wifi`, mDNS hostname `aideck`, and UDP transmit **off** (`src/nina/sdkconfig` around lines 361–368 and 395–406). Wi-Fi TX therefore uses TCP (`src/nina/main/cpx_wifi.c` lines 250–255). Port 5000 for both TCP and UDP (`src/nina/main/cpx_wifi.h` lines 39–43).

**Code.** The NINA image is flashed with OpenOCD, not `idf.py flash`, in the default instructions: partition table at `0x8000`, bootloader at `0x1000`, app at `0x10000` (`src/nina/README.md` lines 22–24). The interface file in that command is `olimex-arm-usb-ocd-h.cfg`. The GAP8 docs recommend the Tiny-H cable and treat OCD-H as an alternative (`src/docker/gapsdk/README.md` lines 44–46). **Opinion:** the two READMEs assume different Olimex JTAG dongles. Pick one and edit the OpenOCD `-f` line; the code does not detect the cable.

### 2.4 Host Python

**Code.** `README.md` lines 24–26 require Ubuntu 22.04 and Python 3.9. `requirements.txt` pins `numpy==1.22.4`, `cflib==0.1.20.1`, `pyzmq==19.0.2`, PyQt5, and an old SciPy.

**Verified here.** On Python 3.12.3, `pip install numpy==1.22.4` fails while building the wheel (`pkgutil.ImpImporter` was removed). **Opinion:** the documented venv has to be Python 3.9. A current distro Python will not install this lockfile as written.

### 2.5 Fragile or undocumented

- GAP and toolchain images clone private-style SSH URLs and are not reproducible without credentials and the patch set. **Code.**
- `himax_start` / `himax_stop` cast the SDK device private pointer to a local `himax_impl_t` and call `pi_cpi_control_start/stop` directly, because "GAP SDK 3.8+ also changes HIMAX_MODE_SELECT" (`src/gap/lib/camera/himax.c` lines 373–401). Any SDK bump that moves fields in that private struct breaks the camera silently. **Code.** **Opinion:** this is the most brittle line in the GAP8 port.
- Root README, NINA README, and `dependencies.lock` disagree on the ESP-IDF version. **Code.**
- `NETWORK_TEST_INPUT` is documented and unused. **Code.**
- `gcc-arm-none-eabi` 13 treats a stock firmware warning as an error. See section 11. **Verified.**

---

## 3. Coroutines and multitasking

### 3.1 What it is

**Code.** All of the mechanism is the header `src/gap/lib/coroutine.h`. There is no separate scheduler thread and no per-task stack. A coroutine is a function that returns at each suspension point and is later called again. The resume point is the `__LINE__` of the macro, stored in `co_fn_ctx_t.resume_point` and dispatched with `switch` (lines 36–41, 207–256).

**Code.** The three states in the paper's Figure 5 are the enum at lines 75–79: `CO_RESUME_START = 0`, `CO_RESUME_RUNNING = -1`, `CO_RESUME_DONE = -2`. `CO_YIELD` (lines 247–250) posts the same function back onto the PMSIS event loop with `pi_task_push`. `CO_WAIT` (lines 253–256) registers the context on a `co_event_t` waiter list and returns. `CO_FN_END` always falls through to `CO_RETURN`, which posts the optional `done_task` (lines 259–275).

**Code.** `co_event_t` is a `pi_task_t` plus a singly linked waiter list (lines 100–105, `src/gap/lib/list.h` lines 39–45). When the event fires, `co_event_callback` pops every waiter and `pi_task_push`es it (lines 143–154). `co_event_wait` brackets the list insert with `disable_irq` / `restore_irq` so the callback cannot run between the insert and the "already done?" check (lines 174–193). If the event is already done, the waiter is still resumed through `pi_task_push`, not inline, so other tasks get a turn. That matches the comment at lines 185–188.

**Code.** The header states the two hard limits (lines 36–41): locals do not survive a suspend, and a `switch` inside a coroutine is illegal because the implementation is itself a `switch`. The `CO_FN_BEGIN` macro turns `-Wuninitialized` into an error to catch the first class of bug (lines 209–213). The example `src/gap/examples/coroutine/main.c` lines 69–80 demonstrates a local that must not be read after `CO_WAIT`.

**Code.** An event group with ANY/ALL waits lives in `src/gap/lib/event_group.h`. It allows one waiter, has no timeout, and `CO_WAIT_MODE_CLEAR` is not implemented (lines 34–37). CPX SPI uses it.

### 3.2 Scheduling model

**Derived.** There is one FC event loop, the PMSIS one (`pmsis_kickoff` in `src/gap/examples/pulp-frontnet/main.c` lines 267–270, then `while (true) pi_yield()` at lines 260–262). Coroutines are callbacks on that loop. They are cooperative: a coroutine that never hits `CO_YIELD` or `CO_WAIT` stalls camera, UART, and SPI. The header says to yield at least every few hundred microseconds (lines 242–246). Nothing enforces that.

**Derived.** Cluster cores do not run these coroutines. `pi_cl_team_fork` inside the generated network is a separate, preemptive-within-the-cluster fork/join. Completion comes back to the FC as a `pi_task`. See section 5.

**Derived, from the public GAP SDK header** `rtos/pulp/pulpos-2/include/pos/data/data.h` at tag `release-v3.8.1` (not compiled in this review): `pi_task_t` is `next` + `arg[4]` + `done` + `id` + `data[8]` + a 4-byte `time`, about **64 bytes** after the `int8_t` padding. `co_fn_ctx_t` embeds one of those, plus function pointer, arg, resume id, done pointer, and a list node (`coroutine.h` lines 83–94). That is on the order of **80 bytes** per live coroutine, and every `co_event_t` embeds another `pi_task_t`. The paper's Appendix B draws `resume_point` as 2 bytes and `resume_task` as a 4-byte pointer and claims **18 bytes per task** and **8×** less memory than a FreeRTOS stack of ≥150 bytes (paper section II-B). **Opinion:** 18 bytes describes the logical resume record in the paper's figure, not the struct that is compiled. The qualitative claim (no private stack, suspend is a function return) is what the code does. The 18-byte and 8× numbers are not measured in this repo, and they do not match this struct.

### 3.3 Interrupts and DMA

**Code.** The only IRQ critical section in the coroutine layer is the short one in `co_event_wait`. Completion of UART, SPI, camera, and cluster work is expressed as `pi_task_push` on a `co_event_t` (`uart.h` lines 52–54, `himax_capture_async` at `himax.c` lines 403–405, `network_run_async` at `network.c` lines 127–131). **Derived:** coroutine bodies run on the FC event loop after the driver posts the task, not inside the DMA ISR. This repo does not contain the PMSIS ISR, so that last sentence is the PMSIS contract plus how this code uses it, not a trace.

**Code.** GPIO trace hooks (`src/gap/lib/trace.h`) can mark camera, UART, CPX, and streamer phases. In the Frontnet config almost every trace id is `TRACE_GPIO_DISABLE` (lines 51–71). **Opinion:** the paper's timing figures were not produced by the configuration that is checked in; someone has to re-enable pins and use a logic analyzer. `tools/trace/` only has an ESP32 notebook.

### 3.4 Cost

**Code.** No cycle counter around `CO_YIELD` is checked in. The paper's "sub-10 µs context switch" is not reproduced here.

**Derived.** The expensive pattern that *is* in the code is the crop loop. `CAMERA_CROP_YIELD` is 1 (`camera.c` line 191), and the loop yields every row (lines 208–218). A 160-row crop is 160 round-trips through the event loop, each of which is a return plus a later indirect call. The `memmove` itself is 160 bytes. **Opinion:** yielding once per row is likely more expensive than the copy, and it is the opposite of "yield every few hundred microseconds" unless a row of bookkeeping somehow costs that much. It is still the right *kind* of hook if the crop would otherwise block SPI.

---

## 4. Camera multi-buffer pipeline

### 4.1 Buffer count and sizes

**Code.** `CAMERA_BUFFERS` is 2 in both real applications (`src/gap/examples/pulp-frontnet/config.h` line 174, `src/gap/examples/streamer/config.h` line 162). The comment says 1 or 2 are supported. `camera_t.frames[CAMERA_BUFFERS]` is a compile-time array (`src/gap/lib/camera.h` lines 54–58).

**Code.** Frontnet's sensor mode is Himax format 2, "HALF", 162×162, 1 byte per pixel (`config.h` lines 122, 265–273). Crop margins are 1 pixel on each side for formats 2 and 3 (lines 183–187). Capture height drops the bottom margin before DMA:

- `CAMERA_CAPTURE_WIDTH` = 162
- `CAMERA_CAPTURE_HEIGHT` = 162 − 1 = 161
- `CAMERA_CROP_WIDTH` = 160, `CAMERA_CROP_HEIGHT` = 160

(`config.h` lines 277–283). **Derived:** one DMA buffer is 162 × 161 = **26082 bytes**. Two of them are **52164 bytes**, and they live inside the streamer payload, not in a separate camera allocation (`src/gap/lib/streamer.c` lines 131–156). The streamer sends only the cropped 160×160 (`streamer.c` lines 159–167 and 205–207). The extra 482 bytes per buffer are the uncropped border plus the bottom row that was never captured. **Opinion:** allocating the uncropped size is correct for the DMA, but it spends L2 on pixels the network never reads (section 7).

**Code.** Clock and rate in the checked-in demo: MCLK mode, 6 MHz, divider /1, auto-exposure off, integration 10 ms, analog gain `0x10` (2×), frame rate **30 Hz** (`config.h` lines 122–159). FC 246 MHz, cluster **175 MHz**, 1.2 V (`config.h` lines 62–64, applied in `src/gap/lib/soc.c` lines 39–45). The paper's Table I row for PULP-Frontnet is 175 MHz and **48 Hz** closed loop. **Code vs paper:** the clock matches; the checked-in frame rate is 30 Hz, not 48. 48 Hz is a legal HALF rate in `src/gap/docs/himax_supported_framerates.md` line 48 ("Actual 48.00 FPS"), so the experiment was a different `HIMAX_FRAME_RATE`, not a different driver.

**Code.** Format 2 at 6 MHz cannot actually hit 150 Hz: the same table says desired 150 FPS becomes 141.18 FPS with a warning (line 55). Format 3 (QQVGA) does hit 150 (line 77). The paper's "up to 150 frame/s at 160×160" (section II-B) is the format-3 number, or the aspirational format-2 number. **Opinion:** the sentence in the paper is stronger than the table the authors checked in. 160×160 at 150 Hz is not what `compute_*` produces for format 2.

### 4.2 DMA and synchronization

**Code.** One capture is `pi_camera_capture_async` into `frame->buffer` for `frame->buffer_size` bytes (`himax.c` lines 403–405). That is the GAP8 CPI plus the microcontroller DMA (µDMA). This file does not program the DMA channel itself.

**Code.** `himax_start` starts the CPI and, if the sensor was in standby, writes streaming mode `0x1` (`himax.c` lines 387–394, mode enum in `himax.h` lines 43–48). `himax_stop` only stops the CPI (lines 396–401). The sensor stays in streaming mode after the first frame. The FC stops the parallel interface between frames.

**Code.** The acquisition coroutine is `camera_task` (`camera.c` lines 128–184). Each loop iteration, for buffer `i mod CAMERA_BUFFERS`:

1. If this buffer was used before, `CO_WAIT` its `done_event` (the consumer must have returned).
2. Arm DMA, `himax_start`, set a trace pin.
3. `CO_WAIT` until DMA completes, `himax_stop`, read the Himax frame counter and `time_get_us()`.
4. Start `camera_crop_task` on the same buffer and wait until the crop posts `done_event`.
5. Start the user callback (`camera_callback` in Frontnet) and **do not wait for it**. The next iteration can arm the other buffer immediately.

**Derived.** This is double buffering of the kind in the paper's Figure 3-B: buffer N can be in `camera_callback` (inference and SPI) while buffer N+1 is being captured. It is not a free-running hardware ring. The CPI is stopped after every frame, so a sensor frame that starts while the CPI is stopped is dropped. The sensor rate (`HIMAX_FRAME_RATE`) and the rate at which the FC re-arms DMA are different knobs. If crop plus scheduling misses the next VSYNC, a frame is skipped even though two buffers exist.

**Code.** Crop is an in-place `memmove` of each row, skipping `CAMERA_CROP_LEFT`, yielding every row (`camera.c` lines 193–222). There is a TODO at lines 199–201: if the crop is only on the bottom, or only a pointer adjustment, the copy could be skipped. It is not skipped. Top and left are 1, so a copy is actually required for this config.

**Code.** The same `frame->consumer_ctx` is reused first for the crop task and then for the user callback (`camera.c` lines 187–189 and 225–227). That is safe only because `camera_task` waits for the crop before starting the callback on that frame. **Derived.**

### 4.3 A real hazard with two buffers

**Code.** `camera_callback` in `src/gap/examples/pulp-frontnet/main.c` lines 83–132 keeps `inference_done`, `streamer_tx_done`, `frame_done`, and `inference_args` in **function-scope statics**. `inference_ctx` is a single global (line 78). `camera_task` can have two callbacks in flight, because it only waits for a buffer's callback when that same buffer is needed again (`camera.c` lines 141–143), and each frame has its own `consumer_ctx`.

**Derived.** Two instances of `camera_callback` then share one set of static events. `CO_WAIT` wakes every waiter (`coroutine.h` lines 149–154). The sequence that the code allows:

- Callback A starts inference and the streamer, then waits on `streamer_tx_done`.
- Callback B starts, sees `camera_started`, and also waits on `streamer_tx_done` if the streamer is still running.
- Both are resumed. B calls `co_fn_push_start` on the same `inference_ctx` and `streamer_send_frame_async` on the same `streamer_t`.

`streamer_send_task` says "Only one send at a time must be inside this critical section, so static variables can be used" (`streamer.c` line 229) and all frames share one `cpx_req` (line 115). `cpx_spi_send_async` aborts if a send is already posted (`cpx_spi.c` lines 234–237). **Opinion:** with `CAMERA_BUFFERS == 2` this is a race, not a hypothetical style issue. It may be rare if inference is so slow that the second callback always runs the "wait until previous is done" loop before the first callback is resumed, but the wakeup rule does not preserve that. A single-buffer build (`CAMERA_BUFFERS 1`) does not have two callback contexts. I did not run the binary, so this is from reading the control flow, not from a crash log.

---

## 5. Cluster compute, and overlap with capture and SPI

**Code.** `cluster_init` opens cluster device 0 and does nothing else (`src/gap/lib/cluster.c` lines 29–42). There is no cluster scheduler in the framework. The network asks for 8 cores:

```20:28:src/gap/examples/pulp-frontnet/app/networks/frontnet-160x32-bgaug/network.mk
CORE ?= 8
...
APP_CFLAGS += -DNUM_CORES=$(CORE)
```

**Code.** `network_run_async` builds a `pi_cluster_task` with master stack 3500 and slave stack 3400 and sends it with `pi_cluster_send_task_to_cl_async` (`network.c` lines 111–131). `execute_layer_fork` (lines 378–416) does `pi_cl_team_fork(NUM_CORES, layerN, ...)` for nine layers. Core 0 allocates 36700 bytes of L1 for the tile buffer (lines 381–382). Convolution, pooling, and the final dense layer all split work by `NUM_CORES` (for example `pulp_nn_conv_u8_u8_i8.c` and `pulp_nn_linear_u8_i32_i8.c`).

**Code.** Cluster DMA is the DORY mchan path in `dory_dma.c`. A comment at lines 120–127 says multi-core 3D DMA deadlocks on GAP8, so `SINGLE_CORE_DMA` forces `dory_dma_memcpy_3d_async` onto core 0. **Opinion:** that is an honest hardware workaround, and it means "8 cores" does not mean 8 cores moving data.

**Code.** Overlap with the camera is explicit. After layer 0, if the caller passed `input_done`, the cluster sends that task back to the FC (`network.c` lines 292–306). Frontnet passes `frame_done` as `input_done` (`main.c` lines 111–116 and 146). The FC callback waits on `frame_done` before it returns the camera buffer (`main.c` lines 129–130). **Derived:** the CPI can refill that buffer while layers 1–8 still run, which is the pipeline in Figure 3-B, provided the callback race in section 4.3 does not fire first.

**Code.** Weights are not in L2 during inference. `network_init` copies eight `.hex` blobs from the read-only filesystem into HyperRAM (`network.c` lines 76–101). `FLASH_WEIGHTS_SIZE` is 311088 (line 52). The L2 working set is `NETWORK_L2_BUFFER_SIZE` 352000 (`network.h` line 30), allocated in `main.c` lines 238–239. **Derived:** 352000 + 2×26082 ≈ 404 kB before code, stacks, and the CPX receive buffer (`CPX_SPI_MTU` is 4092−header, one buffer, `cpx_spi.c` lines 129–135). GAP8 L2 is 512 kB. The paper's Figure 8 is the memory breakdown; the numbers are not repeated in the source. **Opinion:** L2 is tight enough that adding a third camera buffer or turning verbose prints into extra buffers would fail `pi_l2_malloc` at boot. The code already aborts if that malloc fails (`main.c` lines 241–243).

**Code.** The performance printf hardcodes **14138880** MACs (`network.c` line 370). The paper says 14.3 MMAC. **Derived:** 1.4138880×10^7 is the 14.1 MMAC the generator believed, and the paper rounded. Nothing in-tree measures 48 Hz or 3.8 MAC/cycle (Table I). `print_perf` can print MAC/cycle if `NETWORK_VERBOSE` is on (`net_utils.c` lines 35–47), and Frontnet defines `NETWORK_VERBOSE` as 0 (`config.h` line 50).

---

## 6. Inter-MCU transfers and Wi-Fi

### 6.1 STM32 to GAP8: UART, 115200, not CPX

**Code.** Both ends open UART at 115200 (`src/gap/lib/uart.c` lines 40–43, `aideck.c` line 184 `uart1Init(115200)`). The AI-deck task resets the deck GPIO, then blocks in `read_uart_message` (`aideck.c` lines 163–176).

**Code.** STM32 → GAP8 messages are CRC'd packed frames, built in `aideck_protocol.c` lines 62–81:

| Header | Payload | Where defined |
| --- | --- | --- |
| `!STA` | timestamp, position mm, velocity mm/s, acceleration, compressed quaternion, angular rate mrad/s, CRC32 | `aideck_protocol.h` lines 62–91 |
| `!RNG` | 32-bit entropy, CRC32 | lines 97–104 |
| `!TOF` | 8×8 bytes plus resolution, CRC32 | lines 110–120 |

Forwarding runs at 100 Hz (`src/stm32/app/include/frontnet_config.h` lines 90–94) from `frontnet_state_fwd.c` lines 135–156. A 100-deep queue keeps the state that belonged to the exposure timestamp (`STATE_FWD_HISTORY_COUNT`). The author comments a Peek/Receive race against the producer and says it "should almost never be a problem" (`frontnet_state_fwd.c` lines 111–118). **Opinion:** that comment is a real race, left in on purpose. The `ASSERT` on the next line fires if the race breaks the "discarded sample is still old" assumption, which on a flying drone is a fault handler, not a retry.

**Code.** GAP8 → STM32 inference is a different framing. Header bytes are `\x90\x19\x08\x32` on both sides (`uart_protocol.h` lines 91–98, `aideck_protocol.h` lines 48–55). The GAP8 sender writes the header plus the 20-byte `inference_stamped_msg_t` and **no CRC** (`uart_protocol.c` lines 150–155). The STM32 reader uses `size = sizeof(inference_stamped_t)` which is those 20 bytes and no CRC (`aideck_protocol.c` lines 84–86). **Derived:** the asymmetry is consistent. It is also easy to break: adding a CRC on one side only will desynchronize the parser, and the GAP8 resync path drops a byte at a time (`uart_protocol.c` lines 74–88).

**Code.** `input_t.size` is a `uint8_t` (`aideck_protocol.h` line 39). The inference payload is 20, so it fits. A larger message cannot be added without changing that type.

**Code.** The inference callback is deferred to the worker (`aideck.c` line 155, `workerSchedule` in `worker.c` lines 78–90) and the bytes are parsed out of a **single static `buffer[100]`** (`aideck.c` lines 109–110). The worker queue is asynchronous. **Derived:** the next UART message can overwrite `buffer` before the worker runs `inference_stamped_callback`. Frontnet then copies into a queue of length 1 (`frontnet_main.c` lines 72–73 and 190–196), but the copy happens in the worker, not in the UART task. **Opinion:** this is a use-after-reuse bug on the byte buffer. The copy into `frontnet_cmd_t` does not close the window.

**Code.** NINA UART (debug console) is compiled out unless `DEBUG_NINA_PRINT` (`aideck.c` lines 57–89). The paper's STM32↔ESP32 CPX route is not implemented. `src/nina/README.md` lines 29–30 say only GAP↔ESP32↔host is in scope. `frontnet_aideck_protocol.c` line 34 has `TODO: replace this with CPX`.

### 6.2 GAP8 to ESP32: SPI CPX

**Code.** GAP8 is the SPI master, interface 1, CS 0, mode 0, 8-bit (`cpx_spi.c` lines 140–150). Default baud is **30 MHz** GAP8→ESP32 only. Defining `CPX_SPI_BIDIRECTIONAL` drops it to **7.2 MHz** (lines 41–47). The comment and `src/nina/main/config.h` lines 59–62 attribute the cap to swapped MISO/MOSI on the AI-deck, which forces the ESP32 GPIO matrix. Frontnet and the streamer example leave the define **commented out** (`pulp-frontnet/config.h` lines 191–196). The small `examples/cpx` app turns it **on** (`examples/cpx/config.h` line 43).

**Code.** When the define is off, any bytes received in the full-duplex shift are discarded by forcing `receive_req->header.length = 0` (`cpx_spi.c` lines 311–316). **Derived:** the default Frontnet image can stream to the host and cannot receive the host's inference or RTT reply over SPI. The offboard path in `main.c` lines 187–191 (`#ifndef NETWORK_ONBOARD_INFERENCE`) is dead in the default build for two reasons: the onboard define is set (`config.h` line 47) and, even if it were unset, SPI RX is zeroed.

**Code.** Packet layout is a 2-byte length plus a packed CPX routing byte (`cpx_spi.h` lines 33–39, `cpx_types.h` lines 56–63). Max DMA length is 4092, the ESP32 SPI slave DMA limit (line 42). Payloads must be 4-byte aligned; the length check is in bits (`cpx_spi.c` lines 351–363). The streamer pads the last chunk up to a multiple of 4 (`streamer.c` lines 264–267). **Derived:** the pad can read a few bytes past the cropped image. Those bytes are still inside the larger capture allocation (section 4), so this config does not walk off the malloc. A crop that filled the buffer would.

**Code.** The SPI task is a coroutine that waits until both the NINA ready-to-transmit GPIO and a local send or receive request are pending, raises GAP8 RTT, then transfers header, head, and tail (`cpx_spi.c` lines 260–347). Sends are single-flight (`cpx_spi.c` lines 234–237). `cpx_send_task` also waits on a shared `send_done` and the comment says "FIXME: this probably deadlocks if contended" (`cpx.c` lines 127–133). **Opinion:** believe the FIXME. The wait is correct only if waiters wake in queue order and nobody posts `send_done` early. `cpx_start` primes `send_done` with an extra `pi_task_push` (lines 110–112) specifically because there is no mutex API.

**Code.** Routing targets are STM32, ESP32, Wi-Fi host, and GAP8 (`cpx_types.h` lines 34–39). Function ids add `CPX_F_STREAMER = 0x06` on top of the Bitcraze set (lines 41–54). The NINA router forwards by raw buffer, not by parsing the target: SPI payload with non-zero length is handed to Wi-Fi if a client is connected (`src/nina/main/main.c` lines 128–151 and 153–172). **Derived:** a packet addressed to `CPX_T_STM32` would still be sent to the socket. The README's "only GAP↔ESP32↔host" is enforced by not implementing other routes, not by a routing table.

### 6.3 ESP32 Wi-Fi path

**Code.** FreeRTOS tasks, all pinned, priorities in `src/nina/main/config.h` lines 30–52:

| Task | Core | Priority | Role |
| --- | --- | --- | --- |
| LED | 0 | 2 | Blink, also kicks trace sync |
| Wi-Fi status, SPI router, Wi-Fi router | 1 | 3 | Connection state and the two router loops |
| TCP RX, UDP RX | 1 | 4 | Socket receive |
| SPI slave | 1 | TCP/IP priority + 1 | `spi_slave_transmit` loop |
| Trace dump | 1 | 24 | Highest, same core as SPI on purpose (`main.c` lines 190–195) |

The comment at `config.h` lines 31 says ESP-IDF does not want equal priorities on different cores. Several of these share priority 3 or 4 **on core 1**, which is the case the comment allows.

**Code.** SPI RX side has 3 DMA buffers of 4092 bytes and queues of length 3; the TX queue length is 1 (`cpx_spi.c` lines 46–47 and 227–237). Wi-Fi RX has 3 buffers (`cpx_wifi.c` line 54 and 575–581). **Derived:** "multi-buffer" on the ESP32 is 3, not the camera's 2. The paper's "multi-buffer router" (section II-B) matches this, with the small depth. A comment at `cpx_spi.c` line 93 says packet drop should happen before the GAP8 is stalled, and it is marked FIXME. The receive path blocks on `xQueueReceive(free_queue, ...)` (line 93). **Opinion:** under a slow Wi-Fi client the GAP8 SPI master blocks in `CO_WAIT` for SPI-done, which then blocks the streamer, which then holds a camera buffer. Backpressure is real; it is not a drop-on-full router.

**Code.** Default image TX is TCP, port 5000, mDNS `aideck.local`. The host client defaults to sending **replies over UDP** (`src/client/aideck_cpx_streamer/aideck_cpx_streamer/cpx/cpx.py` line 79, `transport/multi.py` lines 33–45) while still receiving both. The ESP32 always starts both RX tasks (`cpx_wifi.c` lines 585–586). **Derived:** camera frames leave by TCP; the small host reply (stats, offboard inference) comes back by UDP. That matches `CONFIG_ENABLE_UDP_TX` unset. It does not match a mental model of "one socket".

**Code.** The paper's "160×160 at 72 Hz, 2.4× the original CPX" and "55 ms mean round-trip" (section II-B) are not constants, tests, or logs in this repo. The mechanisms that would make a higher rate possible are here: 30 MHz SPI, zero-copy head/tail SPI (`cpx_spi.h` lines 48–63; the image bytes are the SPI tail, `streamer.c` lines 249–270), and overlapping SPI and Wi-Fi tasks. **Opinion:** the code can be that fast; this tree does not contain the measurement.

Hardware timestamps: the streamer metadata carries GAP8 microsecond stamps for the frame, the state UART, and the ToF UART, plus the reply stamps used for RTT (`streamer.h` lines 42–77, filled in `streamer.c` lines 202–222). STM32 timestamps in the state message are FreeRTOS ticks, not the GAP8 microsecond clock. **Derived:** "synchronization across MCUs" in the paper is these two clocks sitting in one metadata struct, not a clock-sync protocol. `src/client/.../utils/quatcompress.py` undoes the quaternion. There is no cross-clock offset estimator in the C.

---

## 7. The three in-field applications

The paper's section III has three flights. This repository contains one of them, plus library examples. The paper's Table III says the same thing. **Code and paper.**

### 7.1 What is actually plugged in: human pose, onboard PULP-Frontnet

**Code.** `src/gap/examples/pulp-frontnet/` is the only closed-loop application.

| Piece | Where | What it does |
| --- | --- | --- |
| Sensor | `config.h` | 162×162 Himax, crop to 160×160, 30 Hz, cluster 175 MHz |
| Network | `network.h`, `network.c`, `hex/` | uint8 input of **15360** elements, **4** int32 outputs, 9 layers, 311088 bytes of weights |
| Runtime glue | `main.c` | camera callback → `network_run_async` → dequantize → UART inference, and in parallel `streamer_send_frame_async` |
| STM32 | `src/stm32/app/` | Kalman filter on (x, y, z, φ), velocity controller, 100 Hz setpoint |

**Code.** 15360 = 160×96. Layer 0's generator banner says `x_h 96`, `x_w 160` (`layer0_BNReluConvolution.c` lines 50–51). The directory is named `frontnet-160x32-bgaug`. **Derived:** the tensor is the 160×96 PULP-Frontnet input from the paper (section III-A and Appendix C), not 160×32 and not the 160×160 crop. `network_run_async` reads `activations_size[0]`, which is 15360 (`network.h` lines 64–75), from the start of the camera buffer. The camera buffer after crop is 160×160. **Derived:** the network sees the **top 96 rows** of the 160-row crop and ignores the bottom 64. The paper's "160×96 image" does not say "top-aligned inside a 160×160 capture". **Opinion:** this is probably an accidental ROI. A centered vertical window would be a pointer offset of 32 rows, and that offset is not in `main.c`.

**Code.** Outputs are dequantized with four hard-coded scales (`network.c` lines 418–425) into `x, y, z, phi` (`main.c` lines 152–158) and sent with the STM32 timestamp that was in the latest state message, not the timestamp of the frame (`main.c` line 112 uses `latest_state.timestamp`). **Derived:** if state frames and camera frames are not 1:1, the pose is stamped with a different exposure than the pixels. The STM32 side then looks up the state history by that stamp (`frontnet_main.c` lines 252–266). The lookup is only as good as the stamp GAP8 copied.

**Code.** The controller keeps the drone `horizontalDistance` 1.5 m in front of the subject (`frontnet_config.h` lines 66–71), which matches the paper's 1.5 m. Default altitude target is 1.4 m above the ground. The velocity law is a first-order tracker plus a feedforward of the subject's velocity (`frontnet_ctrl.c` lines 29–37). Limits: 4.8 m/s horizontal, 0.8 m/s vertical, 2.0 rad/s yaw (`frontnet_config.h` lines 74–81). Setpoints are velocity, world frame (`frontnet_ctrl.c` lines 61–71), pushed at priority `COMMANDER_PRIORITY_CRTP + 1` so they override the radio (`frontnet_config.h` lines 83–88). A 100 Hz timer keeps updating between inferences (`frontnet_main.c` lines 314–318 and 376–387). If no inference arrives for 750 ms, it switches to hover (`frontnet_config.h` line 53, `frontnet_main.c` lines 363–368).

**Code.** `minBatteryVoltage` defaults to 2.9 V and is a parameter (`frontnet_config.h` lines 55–56, `frontnet_main.c` line 457). Nothing reads it. The comment says control is disabled below that voltage. **Opinion:** the safety check was never wired up. Do not fly on the assumption that 2.9 V does anything.

**Code.** Weights and one input vector are in the tree as raw files whose names end in `.hex`:

`src/gap/examples/pulp-frontnet/app/networks/frontnet-160x32-bgaug/hex/`

Sizes match `weights_size[]` exactly (1312, 9728, 9728, 19456, 37888, 75776, 149504, 7696) and `inputs.hex` is 15360 bytes. They are not Intel HEX text. **Code** (`network.mk` lines 44–52, `network.h` lines 50–63). There is no PyTorch, no QuantLib recipe, and no training set. **Opinion:** you can flash this network; you cannot retrain it from this repo.

**Code.** Offboard MobileNetV2 from Cereda et al. (paper section III-A, the remote rows of Table II) is not in the client. The GAP8 has a hook that would UART-forward a `STREAMER_TYPE_INFERENCE` blob (`main.c` lines 171–196), and the Python struct exists (`streamer.py` lines 85–93). No host inference script is in `src/client/`. **Opinion:** Table II's remote column is not reproducible from this tree.

### 7.2 Drone-to-drone localization

**Paper.** Section III-B and Appendix C: a fully convolutional net, 160×160 in, three 20×20 maps, onboard, throughputs 10/20/39 Hz, target distance 0.8 m. **Code.** No second network, no 20×20 map decoder, no spiral trajectory, no "observer" controller. Luca Crupi appears as an author of `aideck.c`, not of an application. **Opinion:** this experiment used NanoCockpit, and the application was not part of the release. Table III of the paper already says that.

### 7.3 Nano-drone racing

**Paper.** Section III-C: 162×162 in, three collision probabilities, 331 k parameters, 25 MMAC, 5 Hz and 30 Hz, braking thresholds 0.05 and 0.30. **Code.** Not present. No low-pass on three probabilities, no forward-speed brake. **Opinion:** same as 7.2.

### 7.4 Library examples (not the three flights)

**Code.** `examples/coroutine`, `examples/queue`, `examples/queue_async`, `examples/cpx`, `examples/streamer`. They are the "standalone examples" in Table III. `examples/streamer` is the camera-to-Wi-Fi path without the CNN. `examples/cpx` is the one place bidirectional SPI is on by default.

---

## 8. Paper claims versus this tree

| Claim | Where in the paper | In this repo? |
| --- | --- | --- |
| Coroutine layer over PMSIS, cooperative, no per-task stack | II-B, Appendix B | **Implemented.** `coroutine.h`. The 18 B / sub-10 µs / 8× figures are not measured here, and the struct is larger than 18 B (section 3). |
| Double-buffered camera, streaming mode, up to 150 fps at 160×160 | II-B | **Implemented as a 2-buffer software pipeline** with CPI stop/start between frames. 150 fps at 160×160 is not what format 2 produces at 6 MHz (section 4). Default demo is 30 Hz. |
| Overlap of µDMA, cluster, and SPI | Figure 3-B | **Implemented** by not waiting for the consumer before arming the other buffer, and by releasing the frame after layer 0 (`network.c` 292–306). Subject to the callback race in section 4.3. |
| Zero-copy Wi-Fi stack, 72 Hz of 160×160, 2.4× original CPX, 55 ms mean RTT | II-B | **Mechanisms yes** (SPI tail points at the image, separate SPI and Wi-Fi tasks, 30 MHz). **Numbers no.** Also, default SPI is unidirectional, so the "bi-directional" sentence is an opt-in that the demo does not turn on. |
| PULP-Frontnet 304 k params, 14.3 MMAC, 3.8 MAC/cycle, 175 MHz, 48 Hz inference and closed loop, 0% drop | Table I, III-A | **175 MHz, 8 cores, ~14.14 MMAC constant, 4 outputs, weights on HyperRAM: yes.** 304 k parameters is the right order for 311088 weight bytes but is not a symbol. **48 Hz and 0% drop: not in the source.** Checked-in rate is 30 Hz. MAC/cycle is printed only if verbose is enabled, and it is not. |
| End-to-end latency 30.3 ms at 12/24/48 Hz (Table II) | III-A | **Not a constant.** STM32 logs `inferenceLatency` as tick delta (`frontnet_main.c` lines 243 and 467). Nobody stores 30.3. |
| −30% mean position error | Abstract | **Not computed in the repo.** Table II, which is the only numeric pose table, goes from 0.96 m at 12 Hz to 0.80 m at 48 Hz onboard (**about −17%**), or 1.01 m at 24 Hz to 0.80 m (**about −21%**). Remote MobileNet goes from 0.83 m to 0.65 m (**about −22%**). None of those is −30%. **Opinion:** the abstract number is not the table. It may be a reading of Figure 4 that the prose never states. This repo cannot confirm it. |
| Mission success 40% → 100% | Abstract | **Not in the repo, and not in the body text that was extracted.** Section III-B says the observer loses the target in 2 of 3 runs at 10 Hz (one success out of three) and in 0 runs at the top rate. Section III-C says **60%** success at the highest racing speed, not 100%. **Opinion:** the abstract compresses those plots into a sentence the body does not literally support. Nothing here is a flight log. |
| Three in-field apps on this framework | III, Table III | **One app is in the tree.** The paper's own Table III does not claim the other two were released. |
| Ideal closed loop, "zero overhead due to serialized tasks" | Abstract | **The structure aims at that** (capture of buffer B during inference on buffer A). **Derived:** crop is serial and yields every row, CPI is stopped between frames, and the default build does not stream back over SPI. "Zero overhead" is not something this source can prove. |

**Opinion.** The framework claims (coroutines, double buffer, cluster fork, CPX, UART state forward, Frontnet controller) are real code. The result claims (milliseconds, percents, mission success) live only in the paper. Shipping the framework without the other two apps and without logs is consistent with Table III, and it means a reader cannot audit −30% or 40%→100% from GitHub.

---

## 9. Gaps, likely bugs, hard-coded values

Highest-confidence issues, each labeled.

1. **Two camera callbacks, one set of statics.** Section 4.3. **Derived.** Can trip `cpx_spi_send_async`'s "multiple send" abort or corrupt `cpx_req`.
2. **Inference bytes in a static buffer, consumed later by the worker.** `aideck.c` lines 109–110 and 155. **Derived.**
3. **`compute_frame_len_lines` is called with the arguments swapped.** Definition (`himax.c` line 183) is `(format, line_len_pck, frame_rate)`. Call (line 233) is `(format, desired_frame_rate, line_len_pck)`. **Code.** **Derived:** for an integer frame rate the product `frame_len_lines * real_line_len` still equals `pixel_clock / fps`, so 30 Hz and 48 Hz come out right. The supported-rate table matches that accident (format 2, 30 Hz → 800×250 at 6 MHz, which is what the swapped call computes). A non-integer rate is truncated by the `uint16_t` conversion. **Opinion:** this will confuse the next person who tunes exposure, because `frame_len_lines` is not the number of lines they think it is, even when the fps printout looks correct.
4. **Battery cutoff is a dead parameter.** Section 7.1. **Code.**
5. **State-history race, acknowledged in a comment.** `frontnet_state_fwd.c` lines 111–118. **Code.**
6. **CPX send FIXME about deadlock.** `cpx.c` lines 127–133. **Code.**
7. **SPI RX backpressure FIXME.** `nina/main/cpx_spi.c` line 93. **Code.**
8. **Default SPI drops every received packet.** Section 6.2. The paper's bidirectional stack is off in the demo. **Code.**
9. **Network reads 96 rows of a 160-row buffer, top-aligned.** Section 7.1. **Derived.**
10. **`uart_printf` uses `_uart` which `uart_init` never assigns** (`uart.c` lines 35 and 78–91). **Code.** It is unused by the demo. **Opinion:** dead, and dangerous if someone calls it.
11. **`NETWORK_TEST_INPUT` is unused, GVSOC instructions are not wired.** Section 2.1. **Code.**
12. **ESP-IDF version in the root README is wrong.** Section 2.3. **Code.**
13. **Hard-coded Wi-Fi** `drone_wifi` / `wifi-wifi` in `sdkconfig`. **Code.** Fine for a lab, not a secret, and not runtime-configurable (`src/nina/README.md` lines 16–30).
14. **Command queue length 1.** Inference uses `xQueueOverwrite`, the 100 Hz timer uses `xQueueSend` with timeout 0 (`frontnet_main.c` lines 190–202). **Derived:** a timer tick is dropped if an inference is sitting in the queue, and an inference replaces a pending timer. The task body updates the setpoint on either command, so this mostly coalesces. It is still a single-slot queue on a flying controller.
15. **Missing from the tree, and not bugs so much as scope:** MobileNetV2, the FCNN, the racing CNN, QuantLib, DORY, any flight log, any dataset, any unit test of the coroutine or CPX code. The `hex/` weights for Frontnet **are** present. **Code.**
16. **`src/stm32/docs/kf.pdf`** is a figure, not a derivation of the filter gains. The gains are the literals in `FRONTNET_KF_DEFAULT_CONFIG`. **Code.**

---

## 10. Build and flash on a Crazyflie 2.1 Brushless + AI-deck 1.1

### 10.1 What this firmware supports

**Code.** Device types in the cf2 platform are only:

- `CF20`, Crazyflie 2.0, MPU9250 + LPS25H, `motorMapDefaultBrushed`
- `CF21`, Crazyflie 2.1, BMI088 + BMP388, `motorMapDefaultBrushed`

(`platform_cf2.c` lines 36–50). If the OTP string matches neither, `platformInitConfiguration` returns 1 and `platformInit` aborts (`platform.c` lines 33–41 and 86–94).

**Code.** Brushless PWM exists only as the BigQuad-style path: `BRUSHLESS_MOTORCONTROLLER` is commented out (`src/stm32/crazyflie-firmware/src/config/config.h` line 238), and `ENABLE_ONESHOT125` is off (`motors.h` lines 65–66). `bigquad.c` can print "Switching to brushless." That is a deck, not the Crazyflie 2.1 Brushless product.

**Derived, from Bitcraze's GitHub release notes, not from this tree.** Firmware release **2024.10** (published 2024-10-16) lists "Add platform configuration for CF2.1 brushless" and "Add cf21bl build target". Those files are not in this tree. 2021.06 is three years earlier. **Opinion:** a Crazyflie 2.1 Brushless will not fly this image. If its OTP type is not `CF21`, init aborts. If the type string is still `CF21`, the firmware will drive **brushed** motor pins and look for a **BMI088**, which is the wrong actuator protocol and the wrong IMU for that product. Do not "try it and see"; the motor map is the wrong electrical interface.

**What has to change, opinion, based on the symbols that are absent:**

1. Move `src/stm32/crazyflie-firmware` onto a Bitcraze release that has the `cf21bl` platform (2024.10 or later), including that platform's motor driver and IMU.
2. Re-apply the IDSIA edits that this 2021.06 tree carries: `aideck.c` / `aideck_protocol`, the latest-state queue in `stabilizer.c`, `appInit` → Frontnet, and `DECK_FORCE=bcAI`. The app API around `static_mem.h` and commander priorities has moved since 2021.06; this is a port, not a copy.
3. Do not follow `src/stm32/README.md` lines 11–14 ("flash stock 2021.06 from cfclient so the NRF51 matches") on a brushless airframe. That step is for the brushed aircraft this tree targets. A brushless aircraft ships with a newer radio firmware; downgrading it to 2021.06 is part of the same unsupported combination.
4. The GAP8 and ESP32 images do not know which airframe they are on. They can stay, once the STM32 speaks the same UART protocol. The AI-deck 1.1 (Himax + GAP8 + NINA) is what `BOARD 0` and the NINA pinmux assume (`config.h` on both sides). **Code.**

AI-deck 1.1 itself is the board this code was written for: Himax, GAP8 pads, NINA SPI pins in `src/nina/main/config.h` lines 55–68, deck pid `0x12`. **Code.** **Opinion:** the deck is supported; the brushless flight controller under it is not.

### 10.2 Crazyradio 2.0 and cfclient

**Code.** Host pins are `cflib==0.1.20.1` (`requirements.txt` line 3) and `cflib~=0.1.20.1` (`src/client/crazyflie-clients-python/setup.py`). That cflib release is dated 2022-09-07 on GitHub. Its `cflib/drivers/crazyradio.py` enumerates USB **VID 0x1915, PID 0x7777** only. I fetched the same file at cflib 0.1.28 (2025-02-20) and at current `master`; both still show only PID `0x7777`.

**Code.** Flash command is `make cload`, which runs `python3 -m cfloader ... flash $(PROG).bin stm32-fw` (`crazyflie-firmware/Makefile` lines 441–443). `src/stm32/README.md` lines 28–31 show a `radio://0/100/2M/E7E7E7E7E7` URI. That URI scheme is the CRTP radio in this cflib.

**Opinion.** Crazyradio 2.0 did not exist when cflib 0.1.20.1 was released. I did not confirm its USB PID in this session, so I will not invent one. What I can say: `cfloader` from this pin will only open a dongle that enumerates as `1915:7777`. If Crazyradio 2.0 does not, `make cload` and `cfclient` from this tree will not see it. Use a Crazyradio PA for the documented flow, or replace cflib/cfclient with a version you have personally checked against that dongle. The vendored `cfclient/version.py` still says `2016.02`, which is stale; the pin that matters is `setup.py`.

The documented first-time NRF51 step (flash stock 2021.06 from cfclient) uses that same radio path. **Code,** `src/stm32/README.md` lines 11–14 and root `README.md` lines 38–40.

### 10.3 Steps that match the READMEs, for a brushed Crazyflie 2.1 + AI-deck

These are the steps the repository documents, with the corrections from this review. They were **not** flashed on hardware here.

1. Host: Ubuntu 22.04, **Python 3.9** (not 3.12; section 11). Create the venv from root `README.md` lines 30–36. Install a Crazyradio that this cflib can open (section 10.2).
2. Once per aircraft: from `cfclient`, flash stock **2021.06** so the NRF51 matches, then set a unique radio address (`src/stm32/README.md` lines 11–17). Skip this entire step on a Crazyflie 2.1 Brushless (section 10.1).
3. STM32, from `src/stm32/app`: `make all -j8`, then `make cload` with `CLOAD_ARGS` set to the radio URI. Expect the GCC 13 failure in section 11 unless the toolchain is the older one Bitcraze used around 2021, or the `stringop-overflow` error is demoted. The image is the whole firmware, about 346 kB of flash in the link performed here.
4. GAP8: install the Docker alias from `src/docker/gapsdk/README.md`, `cd src/gap`, `gap8 3.8.1`, `cd examples/pulp-frontnet`, `make clean && make all flash`. JTAG is Olimex Tiny-H in that README. This needs the prebuilt image or an SSH key that can clone `gap_sdk`. Set `HIMAX_FRAME_RATE` if you want the paper's 48 Hz; the file says 30.
5. ESP32: `cd src/nina`, `esp 5.3.3` (not 5.3.1). `idf.py build`. Flash with the OpenOCD line in `src/nina/README.md`, and use the `-f interface/...` that matches the JTAG dongle you actually own. Put the host on SSID `drone_wifi` or change `sdkconfig` and rebuild. UART log is GPIO2 of the NINA, 115200, on the AI-deck's right header (`src/docker/espidf/README.md` lines 100–101, and the UART note added in this fork's README history).
6. Host viewer: `plt_viewer`, which looks for `aideck.local:5000` (`src/client/README.md`). `cfclient` for parameters. Autonomous mode is the parameter `frontnet.enable_control` (`frontnet_main.c` lines 426–429). It does nothing until inferences arrive, and it will not refuse a low battery (section 9).

**Opinion.** Order is STM32 radio link first (so you still have a bootloader), then GAP8 flash, then ESP32 flash, then `cfclient` to confirm the log group `frontnet` is alive before `enable_control`. Flashing the ESP32 or GAP8 does not go through the Crazyradio; both need JTAG (or UART boot for the ESP32, `src/docker/espidf/README.md` lines 113–124).

---

## 11. What was actually built in this environment

The VM is Linux, Python 3.12.3, no Docker, no `arm-none-eabi-gcc` until it was installed for this review, no ESP-IDF, no GAP SDK.

**STM32, verified.**

- Package installed: Ubuntu 24.04 `gcc-arm-none-eabi` 13.2.1 (`15:13.2.rel1-2`).
- `make all -j` in `src/stm32/app` **fails**. `cc1` promotes `stringop-overflow` to an error in stock files:
  - `src/stm32/crazyflie-firmware/src/modules/src/crtp_commander_generic.c` line 336
  - `src/stm32/crazyflie-firmware/src/modules/src/crtp_localization_service.c` line 203
  - both call `quatdecompress` in a way GCC 13 does not like, and the build uses `-Werror`.
- The same sources **link** if that diagnostic is not an error (invoked as `make all CC="arm-none-eabi-gcc -Wno-error=stringop-overflow"`, without editing the tree). Result, copied from the linker printout:

  - `cf2.bin`, Crazyloader build, platform CF2
  - Flash 345616 / 1032192 (33%), text 336708, data 8908
  - RAM 91560 / 131072 (70%)
  - CCM 62916 / 65536 (96%)

- `versionTemplate.py` printed `fatal: No names found, cannot describe anything` and then `Build 0:741f12b (NA) CLEAN`. The firmware directory is not its own git repo, so the version string is whatever `git describe` finds from the parent. **Opinion:** the 2021.06 claim does not appear in the binary's version string.
- Build products were deleted with `make clean` afterward and are not part of the change set.

**Host Python, verified.** `pip install numpy==1.22.4` on Python 3.12 fails while building the wheel. The README's Python 3.9 requirement is real.

**GAP8, not built.** Docker is not installed. The Dockerfile clones `gap_sdk` and the RISC-V toolchain over SSH with a BuildKit secret. This VM has no such key. I did not pull `registry.gitlab.com/eliacereda/gapsdk`. No `pmsis_rules.mk`, so `examples/pulp-frontnet` was not compiled. The `pi_task_t` layout cited in section 3 was read from the public `gap_sdk` tag `release-v3.8.1`, not from a local SDK build.

**ESP32, not built.** Docker is not installed, `idf.py` is not on `PATH`, and a full ESP-IDF 5.3.3 bootstrap was not run. `sdkconfig` and `dependencies.lock` were only read.

**Nothing was flashed. No drone, no Crazyradio, no AI-deck, no GVSOC run.** Latency, 48 Hz, −30%, and mission success were not measured.

---

## 12. Short verdict

**Code.** This is a real, fairly complete implementation of the framework the paper describes: stackless coroutines on PMSIS, a 2-buffer Himax pipeline, an 8-core DORY network with the frame released after the first layer, a UART state/inference protocol, and a CPX SPI-to-Wi-Fi router with 3 DMA buffers. The one shipped application is onboard PULP-Frontnet plus a 1.5 m follow controller. Weights are in the tree.

**Derived.** The checked-in demo is 30 Hz and 160×96-from-the-top-of-a-160×160-buffer, with host-to-GAP8 SPI disabled. Several comment-level FIXMEs are actual concurrency bugs (shared coroutine statics, static UART buffer, swapped Himax arguments that happen to compute integer frame rates).

**Opinion.** The paper's control-performance headlines are not auditable from this repository, and two of the three flights were never part of the release the paper's own Table III describes. A Crazyflie 2.1 Brushless is the wrong flight controller for the vendored 2021.06 firmware; the AI-deck code can stay, the STM32 tree has to move to a `cf21bl` firmware and be ported. Crazyradio 2.0 is not what `cflib==0.1.20.1` was written for.
