# OpenLCC RP2040 Firmware for Lelit Bianca (Gicar V3 Compatibility)

This is a fork of the original [variegated-coffee/open-lcc-rp2040-bianca](https://github.com/variegated-coffee/open-lcc-rp2040-bianca) repository, specifically addressing compatibility issues with the Lelit Bianca V3 Gicar control unit.

Special thanks and respect to **Magnus Nordlander**, whose creation of the OpenLCC unit and original firmware inspired this custom project.

## Project Context
As I do not have a native V3 machine, this firmware was developed and tested on a V1 Bianca that received a V3 Gicar control unit replacement due to a leak. The V3 communication protocol between the Gicar unit and the LCC Front module differs from V1 and V2, which prevents the original V2-based firmware from functioning correctly on V3 hardware.

---

## Key Features & Changes

### Protocol & Communication
* **Command Type 7**: Added `ESP_SYSTEM_COMMAND_SET_STANDBY_MODE` to the communication enum.
* **Status Struct Expansion**: Appended `bool standbyMode` to the packed `ESPSystemStatusMessage` struct.
* **Byte Alignment**: Maintained strict memory packing to ensure UART checksum integrity across the expanded 8-bit payload.
* **Two-Way State Sync**: Integrated `.standbyMode` reporting into the RP2040 status loop to prevent Home Assistant UI toggle "bounce-back."

### Hardware Control & Safety Fixes
* **Triple-Point Kill Switch**: Moved heater overrides to the final `SystemController::loop()` for improved reliability.
* **SSR Queue Protection**: Intercepts outgoing packets to force `brew_boiler_ssr_on` and `service_boiler_ssr_on` to `false` without clearing the `ssrStateQueue`.
* **Pump/Heater Decoupling**: Reconfigured logic to allow full pump and solenoid operation during Standby or "Tank Empty" states while strictly disabling heater power.
* **Stability Patch**: Resolved `BAIL_REASON_SSR_QUEUE_EMPTY` crashes by ensuring the 2.5s timing engine remains populated regardless of heater override status.

### State Machine & Boot Logic
* **Fast Heat-Up (FHU) Bypass**: Modified `handleRunningStateAutomations()` to skip `RUN_STATE_HEATUP_STAGE_1`.
* **Simultaneous Heating**: Enabled immediate transition to `RUN_STATE_NORMAL` from a cold boot. This allows proportional power-sharing between boilers instead of the V3 factory flash-heating sequence (which spikes the brew boiler to 130°C).

### ESPHome Integration
* **LambdaSwitch Expansion**: Updated `OpenLCCBiancaSwitch.h` to instantiate and handle the new Standby Lambda pointer.
* **Python Code-Gen**: Added `CONF_STANDBY_MODE` to the switch component’s `__init__.py`, enabling native YAML support for the new hardware state.

---

## V3 vs. V2 Protocol Memory Map

| Feature | V2 Architecture | V3 Custom Architecture | Change |
| :--- | :--- | :--- | :--- |
| **Status Struct Payload** | Base memory block | Base block + Standby State | **+1 Byte (8 bits)** added to UART block |
| **Standby State Data** | Not implemented | `bool standbyMode;` implemented | **1 Byte allocated** (uses 1 bit) |
| **Command Enum Space** | Max value 6 (`0b00000110`) | New value 7 (`0b00000111`) | **0 Bytes added** (uses unassigned int) |
| **Command Argument** | N/A | Argument 1 (ON) or 0 (OFF) | **0 Bytes added** (reuses `bool1` byte) |
