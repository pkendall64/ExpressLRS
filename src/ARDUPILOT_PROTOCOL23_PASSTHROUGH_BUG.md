# ArduPilot protocol-23 RCIN can reconfigure a UART during serial passthrough

## Status and scope

This note documents a reproducible, mode-dependent ExpressLRS flashing failure and a proposed one-line ArduPilot fix for bench testing before an issue or PR.

- **Observed on hardware:** flashing fails with `SERIALx_PROTOCOL=23`, but succeeds through MAVLink and with protocol `29`, on both ESP32 and ESP8285 receivers.
- **Verified in source and a host harness:** the RCIN polling path can call UART configuration setters while the UART is write-locked for passthrough. The proposed guard prevents those calls in the harness and allows normal reconfiguration after unlocking.
- **Still to verify:** that this guard fixes flashing on the actual FC without regressing RC input or recovery after passthrough. No patched ArduPilot firmware has been built or tested as part of preparing this note.

The proposed change is in **ArduPilot only**. Do not modify ExpressLRS `lib/SerialUpdate/stub_flasher.cpp`, its wire protocol, or the uploader while testing this patch.

## Hardware and observed results

- Flight controller: **SpeedyBee F405 Wing**.
- Flight-controller firmware: **ArduPilot Plane 4.7.1**.
- ESP8285 receiver: **RadioMaster RP2**.
- ESP32 receiver: tested successfully/unsuccessfully as below; exact receiver model has not been recorded.

| Receiver family | FC serial configuration | Flashing result | Note |
| --- | --- | --- | --- |
| ESP32 | MAVLink | Succeeds | Exact MAVLink protocol parameter value not recorded |
| ESP32 | `23` — RCIN, receiver using CRSF | Fails | Documented configuration for CRSF RC input |
| ESP32 | `29` — Crossfire VTX | Succeeds | No RC input in this configuration |
| ESP8285 / RP2 | MAVLink | Succeeds | Repeated successful uploads once initial detection completes |
| ESP8285 / RP2 | `23` — RCIN, receiver using CRSF | Fails | Some flash packets succeed before failure |
| ESP8285 / RP2 | `29` — Crossfire VTX | Succeeds | No RC input in this configuration |

The reported RP2 error is `C100`, which the esptool 4.2.1 flasher stub uses for a data-checksum failure (`ESP_BAD_DATA_CHECKSUM = 0xC1`, followed by a zero status byte). This indicates that the received flash data failed the stub's checksum check; it does **not**, by itself, prove a buffer overflow or identify the corruption point. See the [matching upstream stub error definitions](https://github.com/espressif/esptool/blob/v4.2.1/flasher_stub/include/stub_flasher.h).

Host-write pacing and smaller-packet experiments did not resolve the failure. Switching the ESP8285 upload to 460800 baud after stub startup also did not resolve it. Those experiments should not be repeated or varied during the lock-guard test.

### Protocol 29 is diagnostic evidence, not a receiver workaround

[ArduPilot documents protocol `23` for CRSF/ELRS RC input and `29` for VTX-only CRSF operation](https://ardupilot.org/plane/docs/common-tbs-rc.html). Missing RC input on `29` is therefore not evidence of a separate RC-input bug.

The useful observation is that removing the protocol-23 RCIN path permits flashing. Keep protocol `23` for the receiver when testing the proposed fix; do not present `29` as a normal RC configuration.

## Exact upstream revision

The inspected `Plane-4.7.1` tag resolves to:

```text
dbe792162d06cab66c3475fd5556bf7a120f119e
```

The links below are pinned to that commit, rather than to a moving branch. Current `master` has not been assessed by this note; check it before preparing a PR.

## Where the bug appears in ArduPilot

| Location | Relevant behavior |
| --- | --- |
| [`GCS::passthru_timer()` — GCS_Common.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/GCS_MAVLink/GCS_Common.cpp#L7269-L7344) | Locks both passthrough ports and forwards bytes with `read_locked()` / `write_locked()`. |
| [`AP_RCProtocol::check_added_uart()`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_RCProtocol/AP_RCProtocol.cpp#L359-L405) | Checks whether a UART exists, but not whether it is locked. Can mark the RC UART for reopening after receiver inactivity, then reapply its configuration on a later call. |
| [`AP_RCProtocol::SerialConfig::apply_to_uart()`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_RCProtocol/AP_RCProtocol.cpp#L326-L336) | Calls `configure_parity()`, `set_stop_bits()`, `set_options()`, and finally `begin()`. |
| [`AP_RCProtocol_CRSF::is_rx_active()` — AP_RCProtocol_CRSF.h](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.h) | Considers the CRSF receiver inactive when valid receive traffic has stopped for the configured timeout, 150 ms in this revision. |
| [`ChibiOS::UARTDriver::configure_parity()`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_HAL_ChibiOS/UARTDriver.cpp#L1409-L1469) and [`set_stop_bits()`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_HAL_ChibiOS/UARTDriver.cpp#L1474-L1504) | Stop and restart the serial driver with `sdStop()` / `sdStart()`. Neither checks the passthrough lock, and even reapplying the same settings restarts the UART. |
| [`AP_HAL::UARTDriver` lock handling](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_HAL/UARTDriver.cpp) and [`is_write_locked()`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_HAL/UARTDriver.h#L228-L233) | Ordinary `begin()` and reads/writes respect the lock. A public lock-query method already exists, so no new HAL API is needed. |

### Failure sequence

1. The receiver runs normal CRSF firmware and the FC uses protocol `23` for RC input.
2. ArduPilot enables serial passthrough and locks the USB and receiver UARTs. Ordinary RC reads no longer consume bytes from the receiver port.
3. The receiver enters its ROM bootloader or embedded flasher and stops producing CRSF traffic. The RC backend no longer sees valid receiver frames.
4. `check_added_uart()` can set `added.opened = false` after inactivity and the existing configuration-age threshold. On a subsequent call, `apply_to_uart()` runs again. There is also an autodetection configuration-cycling path when the frontend is searching.
5. The parity and stop-bit setters stop/start the UART **before** `begin()` is called. The lock blocks `begin()`, but that is too late to prevent the earlier hardware reconfiguration.

**[INFERENCE]** Restarting the serial driver during a flash packet can explain the partial transfers and checksum errors. The source and cross-chip test matrix support this explanation, but the actual UART interruption has not been captured on the FC. A successful, otherwise unchanged patched-firmware test would provide the next important evidence.

This is a configuration-ownership issue, not evidence that ordinary CRSF telemetry writes bypass `write_locked()` or leak into the transfer.

## Minimal patch to test first

File: `libraries/AP_RCProtocol/AP_RCProtocol.cpp`

Return before RC scanning, handshaking, or configuration-state changes while the UART is write-locked. Passthrough takes both read and write locks, so the existing write-lock query covers this case.

Save this block as `protocol23-passthrough.patch` in the ArduPilot checkout:

```diff
--- a/libraries/AP_RCProtocol/AP_RCProtocol.cpp
+++ b/libraries/AP_RCProtocol/AP_RCProtocol.cpp
@@ -358,7 +358,7 @@
 
 void AP_RCProtocol::check_added_uart(void)
 {
-    if (!added.uart) {
+    if (!added.uart || added.uart->is_write_locked()) {
         return;
     }
     uint32_t now = AP_HAL::millis();
```

Why this location:

- Covers all callers of the shared RCIN polling function, including its configuration-retry path.
- Prevents `added.opened` from being set as if the RC backend had successfully reclaimed a locked UART.
- Keeps ordinary unlocked RC discovery and receiver-reconnection behavior intact.
- Reuses the existing HAL ownership API; no protocol-specific baud workaround or new parameter is needed.

### Existing upstream precedent

ArduPilot already applies this design in [`AP_GPS::update_instance()`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_GPS/AP_GPS.cpp): it returns before GPS detection, configuration, and timeout-driven reinitialization when another driver has locked the port.

```cpp
if (locked_ports & (1U<<instance)) {
    // the port is locked by another driver
    return;
}
```

GPS uses its own `locked_ports` mask, not the HAL `is_write_locked()` query. The precedent is the ownership rule and entry-point suspension, not an identical lock implementation. This is verified existing upstream code, not a separately researched historical fix commit.

The proposed RCIN patch follows that same rule: suspend the normal protocol's port-processing path while another owner is using the UART. This supports presenting the change as a correction to existing ownership handling rather than an ExpressLRS-specific workaround.

**Do not add an unconditional lock guard to every ChibiOS configuration setter as the first fix.** The legitimate passthrough owner itself changes parity while holding the lock. A blanket setter guard could block that operation without distinguishing owners.

### Scope and concurrency caveat

This is a minimal candidate for the reported protocol-23 path, not a claim that every UART configuration path is now protected. The VTX-only CRSF path has a separate [`AP_RCProtocol_CRSF::start_uart()`](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.cpp) which also changes UART settings; any broader hardening should be reviewed separately rather than mixed into the first A/B test.

The early return also does not make lock acquisition and reconfiguration one atomic transaction. It prevents repeated RCIN activity while the lock is already held, which is the failure mechanism being tested. Review the handover race with ArduPilot maintainers before describing the patch as a complete ownership redesign.

## Validation already performed for this patch

The exact diff above was checked with `git apply --check` and applied successfully to a temporary copy of the release-pinned source.

A temporary C++ harness compiled the release's actual `check_added_uart()` and `SerialConfig::apply_to_uart()` functions against a recording UART boundary. With an inactive CRSF backend and the UART locked:

```text
Unpatched: 4 UART setter calls across two reconfiguration cycles
Patched:   0 UART setter calls
```

The harness also confirmed that:

- UART reconfiguration resumes after unlocking.
- A locked UART awaiting initial configuration is not marked opened.
- A null UART remains safe.

This is a source-level behavior check, **not** a full ArduPilot build, a ChibiOS register-level test, or proof that physical flashing is fixed. The temporary harness did not modify either repository's firmware sources.

## Build a test firmware for SpeedyBee F405 Wing

Use a separate ArduPilot checkout, not the ExpressLRS repository. Back up the FC's parameters and retain the known-good FC firmware before installing anything experimental. Remove the propeller/disconnect propulsion; keep these tests disarmed and on the bench.

### 1. Check out the tested release

```sh
git clone --branch Plane-4.7.1 --recurse-submodules https://github.com/ArduPilot/ardupilot.git ardupilot-protocol23
cd ardupilot-protocol23
git switch -c fix/rcin-passthrough-lock
git rev-parse HEAD
```

The final command should print `dbe792162d06cab66c3475fd5556bf7a120f119e`. Starting at the same release keeps this experiment from also changing unrelated flight-controller behavior.

### 2. Install the supported build prerequisites

Follow ArduPilot's official [macOS setup](https://ardupilot.org/dev/docs/building-setup-mac.html) or [Linux/Ubuntu setup](https://ardupilot.org/dev/docs/building-setup-linux.html), using the appropriate ARM cross-compiler. The ExpressLRS PlatformIO environment is not the ArduPilot build environment.

The commands below follow the release's [BUILD.md](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/BUILD.md); they have not been executed as a full board build here.

### 3. Apply only the lock guard and build Plane

After saving the diff above in `protocol23-passthrough.patch`:

```sh
git apply --check protocol23-passthrough.patch
git apply protocol23-passthrough.patch
git diff --check
./waf configure --board SpeedyBeeF405WING
./waf plane
```

`SpeedyBeeF405WING` is the case-sensitive [board definition name](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/hwdef.dat). The application firmware artifact is expected at:

```text
build/SpeedyBeeF405WING/bin/arduplane.apj
```

Install that application image using your normal custom-firmware upload procedure. This test does not require replacing the FC bootloader. Verify the running FC firmware after installation, and retain the build log and exact patch because an uncommitted patch alone does not give the build a new Git commit ID.

## Bench test and acceptance criteria

Use the existing failure logs as the baseline. Keep the same receiver image, uploader version/options, wiring, power arrangement, serial options, and baud choices; change only the ArduPilot guard.

1. **Record configuration.** Save the receiver-connected `SERIALx_PROTOCOL`, `SERIALx_BAUD`, `SERIALx_OPTIONS`, `RC_OPTIONS`, and `RC_PROTOCOLS`, plus the passthrough parameters. Use the actual receiver port from the setup log; do not assume a UART number from the board model.
2. **Confirm normal RC first.** With the receiver in CRSF mode and that FC port set to `23`, confirm valid RC input before entering passthrough. Do not change to protocol `29` for this test.
3. **Repeat the previously failing path.** Run at least five complete protocol-23 uploads on each available receiver family, starting from normal receiver firmware each time. Record successes/failures rather than only the best run. Require completion and the uploader's final verification, not just successful SYNC or initial flash progress.
4. **Check RC recovery.** Confirm RC input after the upload and after a cold restart. Also test recovery when passthrough releases its lock without rebooting the FC: close the uploader and allow the configured inactivity timeout to expire, or disable passthrough through a separate working MAVLink connection. The USB port cannot simultaneously carry normal MAVLink commands while it is locked for raw passthrough.
5. **Check an independent control.** Confirm the already-working MAVLink upload path still succeeds, without introducing new uploader changes.
6. **Check ordinary receiver recovery.** With passthrough off, power-cycle the receiver and verify that normal RC acquisition/reconnection still works. RC discovery must not remain suspended after the UART unlocks. Loss of RC while deliberately running the receiver flasher is expected; this is not a flight-safe operating state.
7. **If flashing still fails, retain the first failure.** Capture the last successful command/block and first failure with timestamps. Keep direct-UART receiver recovery available: an interrupted receiver flash can prevent normal firmware from starting.

Do not return the aircraft to service on the basis of flashing tests alone. Restore or fully validate the flight firmware/configuration and check normal RC/failsafe behavior before flight.

### Optional instrumentation if the result is inconclusive

Count attempted RC UART reconfigurations while `is_write_locked()` is true, and correlate them with the passthrough interval. Record via an independent debug connection, retained counters, or another non-passthrough channel. **Do not print diagnostics to either passthrough UART**, because that would itself contaminate the byte stream and invalidate the test.

A logic-analyzer capture of FC-TX to receiver-RX can help distinguish dropped bytes from malformed bytes. Neither this instrumentation nor receiver-stub changes are required for the initial guard-only test.

## Material for an upstream issue or PR

Suggested title:

> AP_RCProtocol: suspend protocol-23 RCIN polling while its UART is locked for passthrough

Include:

- FC model, exact firmware revision, receiver models, and the six-case matrix above.
- The receiver port's actual parameters and the complete upload invocation, including initial/final baud choices.
- Existing unpatched failure logs and patched success/failure counts for both receiver families.
- Results for normal RC input, receiver reconnection, and passthrough-unlock recovery.
- The pinned source links and minimal patch, with the host-harness/physical-test distinction intact.
- Any counter or logic-analyzer evidence tying a UART restart to the failed transfer.

Check for an existing report and whether current `master` still contains the unguarded path before opening a PR. Keep full esptool traces private or sanitize them appropriately: they include firmware data and may contain embedded configuration or credentials.
