# OpenLCC RP2040 Firmware for Lelit Bianca (Gicar V3 Compatibility)

This is a fork of the original [variegated-coffee/open-lcc-rp2040-bianca](https://github.com/variegated-coffee/open-lcc-rp2040-bianca) repository, specifically addressing compatibility issues with the Lelit Bianca V3 Gicar control unit.

Special thanks and respect to **Magnus Nordlander**, whose creation of the OpenLCC unit and original firmware inspired this custom project.

## Project Context
As I do not have a native V3 machine, this firmware was developed and tested on a V1 Bianca that received a V3 Gicar control unit replacement due to a leak. The V3 communication protocol between the Gicar unit and the LCC Front module differs from V1 and V2, which prevents the original V2-based firmware from functioning correctly on V3 hardware.

---

## Key Features & Changes

### Protocol & Communication
* **Standby Command**: Added `ESP_SYSTEM_COMMAND_SET_STANDBY_MODE` as the **last** entry of the command enum, so all existing command numbers stay unchanged.
* **Status Struct Expansion**: Appended `bool standbyMode` to the **end** of the packed `ESPSystemStatusMessage` struct.
* **Shared Header**: `src/esp-protocol.h` must stay byte-identical to `custom_components/open_lcc_bianca/esp-protocol.h` in the ESPHome repo. Only ever add new fields/commands at the end.
* **Byte Alignment**: Maintained strict memory packing to ensure UART checksum integrity across the expanded 8-bit payload.
* **Two-Way State Sync**: Integrated `.standbyMode` reporting into the RP2040 status loop to prevent Home Assistant UI toggle "bounce-back."

### Hardware Control & Safety Fixes
* **Triple-Point Kill Switch**: Moved heater overrides to the final `SystemController::loop()` for improved reliability.
* **SSR Queue Protection**: Intercepts outgoing packets to force `brew_boiler_ssr_on` and `service_boiler_ssr_on` to `false` without clearing the `ssrStateQueue`.
* **Pump/Heater Decoupling**: Reconfigured logic to allow full pump and solenoid operation during Standby or "Tank Empty" states while strictly disabling heater power.
* **Stability Patch**: Resolved `BAIL_REASON_SSR_QUEUE_EMPTY` crashes by ensuring the 2.5s timing engine remains populated regardless of heater override status.

### State Machine & Boot Logic
* **Cold-Start Heat-Up (unchanged from the original firmware)**: If the brew set point is above 80 °C and the brew boiler is below 65 °C at start-up, the firmware still runs the original two-stage heat-up: Stage 1 heats the brew boiler towards 130 °C with the service boiler off; once it passes 128 °C, Stage 2 holds 130 °C for 4 minutes with the service boiler heating normally; then it switches to the normal set points (`RUN_STATE_NORMAL`). A warm start (brew boiler at or above 65 °C) goes straight to normal operation. The Fast Heat-Up bypass is **not** implemented in this firmware.
* **Sleep & Standby Persistence**: Sleep and Standby are saved to the settings flash and restored after a reboot or power cut. The original firmware cleared Sleep on a cold boot.

### Low-Flow Pre-infusion Routine
* Triggered from ESPHome (`open_lcc.bianca.enqueue_routine`) when a brew starts and the "Low Flow Pre-infusion" switch is on.
* Sequence: low flow until 6 s of brew time → full flow for 14 s → low flow until the lever is closed.
* When the brew ends (or the routine is cancelled) the flow is reset to full flow, so the next brew never starts in low flow by accident.

### ESPHome Integration
* **LambdaSwitch Expansion**: Updated `OpenLCCBiancaSwitch.h` to instantiate and handle the new Standby Lambda pointer.
* **Python Code-Gen**: Added `CONF_STANDBY_MODE` to the switch component’s `__init__.py`, enabling native YAML support for the new hardware state.

---

## V3 vs. V2 Protocol Memory Map

| Feature | V2 Architecture | V3 Custom Architecture | Change |
| :--- | :--- | :--- | :--- |
| **Status Struct Payload** | Base memory block | Base block + Standby State | **+1 Byte (8 bits)** added to UART block |
| **Standby State Data** | Not implemented | `bool standbyMode;` implemented | **1 Byte allocated** (uses 1 bit) |
| **Command Enum Space** | Last value 12 (`CLEAR_ROUTINE`) | New value 13 (`SET_STANDBY_MODE`, appended last) | **0 Bytes added** (existing values unchanged) |
| **Command Argument** | N/A | Argument 1 (ON) or 0 (OFF) | **0 Bytes added** (reuses `bool1` byte) |

## Contributors
* **Magnus Nordlander**: Original Creator.
* **Andrey K.**: V3 Protocol implementation and stability fixes (2026-05-05).
