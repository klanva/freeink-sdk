#pragma once

// FreeInk SDK — boot-time recovery hatch.
//
// The stock Xteink (and most ESP32) second-stage bootloader can't read buttons —
// it just boots whatever otadata selects. So "hold a combo at reset to fall back
// to the recovery firmware" can only be honoured by the firmware that actually
// boots. This is that check, made shareable: call recovery::checkBootCombo() as
// the VERY FIRST thing in setup() of every firmware you want to be escapable
// (the recovery flasher itself, the editor, the reader, ...).
//
// Convention: the recovery / "escape hatch" firmware lives in OTA slot 0 (ota_0,
// upload offset 0x10000). On Xteink X4 Pro (ESP32-S3), holding the Right hardware button
// (GPIO 7, active-LOW with internal pull-up) during startup repoints otadata at slot 0
// and reboots into it.
//
// It is always safe to call unconditionally and early — it does nothing unless
// ALL of these hold: the combo is pressed, slot 0 contains a valid app image (magic 0xE9),
// and the caller isn't already running from slot 0 (preventing infinite reboot loops).
// When it does act, it reboots into Escape Hatch and never returns.
//
// What it can't do: escape a firmware that crashes in ROM / early SDK init before
// this call is reached. (A corrupt app image is still caught for free — the
// bootloader falls back to the other OTA slot on its own.) A truly unconditional
// GPIO recovery would require a custom second-stage bootloader, which the recovery
// firmware deliberately never reflashes.

namespace freeink {
namespace recovery {

// Read the recovery combo and, if held, switch to OTA slot 0 and reboot. Returns
// immediately (no reboot) in every other case.
void checkBootCombo();

}  // namespace recovery
}  // namespace freeink
