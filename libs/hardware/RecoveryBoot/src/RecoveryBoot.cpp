#include "RecoveryBoot.h"

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_rom_crc.h>
#include <esp_system.h>
#include <spi_flash_mmap.h>
#include <string.h>

#if __has_include(<Logging.h>)
#include <Logging.h>
#else
#define LOG_INF(tag, fmt, ...)
#define LOG_ERR(tag, fmt, ...)
#endif

namespace freeink {
namespace recovery {
namespace {

// --- Hardware Recovery Combo for Xteink X4 Pro (ESP32-S3) -------------------
// The physical Right button is wired to GPIO 7 (active-LOW with internal pull-up).
// During cold boot or reset, holding this button triggers an immediate fallback
// to the emergency Escape Hatch micro-bootloader partition (ota_0).
//
// Fast-path: On normal boots, GPIO 7 is unpressed (reads HIGH). We exit in < 1 ms so
// standard boot time is completely unaffected.
//
// Debounce: If GPIO 7 reads LOW, we verify continuous hold for 30-50 ms
// (7 consecutive LOW samples spaced by 6 ms = 42 ms). Any HIGH read aborts.
constexpr uint8_t kRecoveryPin = 7;
constexpr int kConsecutiveSamples = 7;
constexpr int kSampleIntervalMs = 6;

bool comboHeld() {
  pinMode(kRecoveryPin, INPUT_PULLUP);
  delayMicroseconds(50);  // Allow internal pull-up to settle

  // Fast-path check: not held -> return immediately (<1ms)
  if (digitalRead(kRecoveryPin) == HIGH) {
    return false;
  }

  // Pin sampled LOW; verify stable physical hold with 30-50 ms debouncing
  for (int i = 0; i < kConsecutiveSamples; ++i) {
    delay(kSampleIntervalMs);
    if (digitalRead(kRecoveryPin) != LOW) {
      return false;  // Contact bounce, glitch, or released early
    }
  }
  return true;
}

// --- otadata low-level sector switch ----------------------------------------
// Point the ESP32 second-stage bootloader at `dest` by writing a fresh otadata
// record into the inactive slot. Bypasses runtime esp_image_verify (which
// rejects patched Xteink images on USB-locked hardware). Layout per
// esp_flash_partitions.h; CRC32-LE covers ota_seq (4 bytes) only.
struct __attribute__((packed)) SelectEntry {
  uint32_t ota_seq;
  uint8_t seq_label[20];
  uint32_t ota_state;
  uint32_t crc;
};
static_assert(sizeof(SelectEntry) == 32, "SelectEntry must be 32 bytes");

constexpr uint32_t kOtaImgNew = 0;      // ESP_OTA_IMG_NEW
constexpr uint32_t kOtaImgInvalid = 3;  // ESP_OTA_IMG_INVALID
constexpr uint32_t kOtaImgAborted = 4;  // ESP_OTA_IMG_ABORTED

#ifndef SPI_FLASH_SEC_SIZE
#define SPI_FLASH_SEC_SIZE 4096
#endif

uint32_t seqCrc(uint32_t seq) {
  return esp_rom_crc32_le(UINT32_MAX, reinterpret_cast<const uint8_t*>(&seq), sizeof(uint32_t));
}

// A partition begins with a plausible app image (magic 0xE9). Excludes an erased
// (0xFF) / empty slot. Deliberately not esp_image_verify (see above).
bool hasApp(const esp_partition_t* p) {
  if (!p) return false;
  uint8_t magic = 0;
  return esp_partition_read(p, 0, &magic, sizeof(magic)) == ESP_OK && magic == 0xE9;
}

bool switchTo(const esp_partition_t* dest) {
  if (!dest) return false;
  const esp_partition_t* otadata =
      esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, nullptr);
  if (!otadata || otadata->size < 2 * SPI_FLASH_SEC_SIZE) {
    LOG_ERR("RECOVERY", "otadata partition missing or smaller than 2 sectors");
    return false;
  }

  SelectEntry slots[2] = {};
  if (esp_partition_read(otadata, 0, &slots[0], sizeof(SelectEntry)) != ESP_OK ||
      esp_partition_read(otadata, SPI_FLASH_SEC_SIZE, &slots[1], sizeof(SelectEntry)) != ESP_OK) {
    LOG_ERR("RECOVERY", "Failed to read otadata sectors");
    return false;
  }

  // Active slot = valid CRC, highest seq, not INVALID/ABORTED.
  int activeIdx = -1;
  uint32_t activeSeq = 0;
  for (int i = 0; i < 2; ++i) {
    if (slots[i].ota_seq == 0xFFFFFFFFu) continue;
    if (slots[i].crc != seqCrc(slots[i].ota_seq)) continue;
    if (slots[i].ota_state == kOtaImgInvalid || slots[i].ota_state == kOtaImgAborted) continue;
    if (activeIdx < 0 || slots[i].ota_seq > activeSeq) {
      activeIdx = i;
      activeSeq = slots[i].ota_seq;
    }
  }

  // ota_seq encoding: (seq - 1) % NUM_OTA_PARTITIONS selects the partition.
  const uint32_t destIdx =
      static_cast<uint32_t>(dest->subtype) - static_cast<uint32_t>(ESP_PARTITION_SUBTYPE_APP_OTA_0);
  if (destIdx > 15) {
    LOG_ERR("RECOVERY", "Destination partition subtype is not an OTA slot: 0x%02x", dest->subtype);
    return false;
  }

  // Smallest seq > activeSeq landing on `dest` (2 OTA partitions).
  uint32_t newSeq = activeSeq + 1;
  while (((newSeq - 1u) % 2u) != (destIdx % 2u)) ++newSeq;

  SelectEntry next = {};
  next.ota_seq = newSeq;
  memset(next.seq_label, 0xFF, sizeof(next.seq_label));
  next.ota_state = kOtaImgNew;
  next.crc = seqCrc(next.ota_seq);

  // Write the OTHER otadata slot so the bootloader sees the higher seq there.
  const int targetSlot = (activeIdx == 0) ? 1 : 0;
  const size_t targetOff = static_cast<size_t>(targetSlot) * SPI_FLASH_SEC_SIZE;
  if (esp_partition_erase_range(otadata, targetOff, SPI_FLASH_SEC_SIZE) != ESP_OK) {
    LOG_ERR("RECOVERY", "Failed to erase otadata slot %d", targetSlot);
    return false;
  }
  if (esp_partition_write(otadata, targetOff, &next, sizeof(next)) != ESP_OK) {
    LOG_ERR("RECOVERY", "Failed to write otadata slot %d", targetSlot);
    return false;
  }

  LOG_INF("RECOVERY", "otadata switch: wrote slot %d, seq %u, crc 0x%08x -> %s",
          targetSlot, static_cast<unsigned>(newSeq), static_cast<unsigned>(next.crc),
          dest->label ? dest->label : "ota_0");
  return true;
}

}  // namespace

void checkBootCombo() {
  if (!comboHeld()) {
    return;
  }

  // Recovery firmware / Escape Hatch micro-bootloader lives in ota_0 (offset 0x10000).
  const esp_partition_t* hatch =
      esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
  if (!hatch) {
    LOG_ERR("RECOVERY", "Escape Hatch partition ota_0 not found");
    return;
  }

  // Prevent infinite reboot loops if already executing from ota_0.
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running && running->address == hatch->address) {
    LOG_INF("RECOVERY", "Already running from ota_0 (0x%08x); skipping recovery reboot",
            static_cast<unsigned>(running->address));
    return;
  }

  // Verify that ota_0 contains a valid executable application image (0xE9 magic byte).
  if (!hasApp(hatch)) {
    LOG_ERR("RECOVERY", "ota_0 at 0x%08x does not contain valid application image (missing 0xE9 magic)",
            static_cast<unsigned>(hatch->address));
    return;
  }

  LOG_INF("RECOVERY", "Hardware recovery combo held! Switching otadata to ota_0 (0x%08x)...",
          static_cast<unsigned>(hatch->address));

  if (switchTo(hatch)) {
    LOG_INF("RECOVERY", "Successfully switched otadata to ota_0. Restarting system...");
    delay(50);
    esp_restart();
  } else {
    LOG_ERR("RECOVERY", "Failed to switch otadata to ota_0");
  }
}

}  // namespace recovery
}  // namespace freeink
