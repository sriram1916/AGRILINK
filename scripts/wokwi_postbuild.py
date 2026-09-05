# Post-build hook for the Wokwi simulation.
#
# PlatformIO/ESP-IDF invoke esptool `merge_bin` to produce firmware.bin but
# place the bootloader at file offset 0x0000 (no 0xFF padding before it).
# Wokwi's ESP32 emulator expects the bootloader at flash offset 0x1000 with
# 0xFF padding, otherwise the ROM cannot find the second-stage bootloader
# and the application never starts -- producing a blank serial monitor.
#
# Additionally, Wokwi's canonical ESP-IDF integration uses
# `flasher_args.json` (rather than a raw merged `.bin`) because the
# extension parses `flash_files` to load each artifact at its declared
# flash offset, bypassing auto-detection entirely. This script therefore:
#
#   1. Regenerates `firmware.bin` with the correct flash layout
#      (bootloader @0x1000, partition @0x8000, app @0x10000, 0xFF pad).
#   2. Generates a standalone `app.bin` from `firmware.elf` via
#      `esptool elf2image` (the canonical app image, with the 0xE9
#      header and SHA256 appended).
#   3. Writes a corrected `flasher_args.json` that references the
#      existing bootloader.bin, partitions.bin, and app.bin so that
#      Wokwi loads the complete ESP-IDF image with explicit offsets.

import os
import shutil
import subprocess

Import("env")

PROJECT_DIR = env["PROJECT_DIR"]
BUILD_DIR = os.path.join(PROJECT_DIR, ".pio", "build", "esp32dev")

ESPTOOL_PY = None
candidates = [
    os.path.join(PROJECT_DIR, ".pio", "packages", "tool-esptoolpy", "esptool.py"),
    os.path.expanduser("~/.platformio/packages/tool-esptoolpy/esptool.py"),
    "C:\\Users\\Vamshi\\.platformio\\packages\\tool-esptoolpy\\esptool.py",
]
for c in candidates:
    if os.path.isfile(c):
        ESPTOOL_PY = c
        break

if ESPTOOL_PY is None:
    print("wokwi_postbuild: esptool.py not found, skipping firmware rewrite")
    Return()


def _run(cmd):
    print("wokwi_postbuild: running: " + " ".join(cmd))
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(result.stdout)
    return result.returncode


def _rewrite_firmware_bin(source, target, env):
    bootloader_bin = os.path.join(BUILD_DIR, "bootloader.bin")
    partitions_bin = os.path.join(BUILD_DIR, "partitions.bin")
    app_bin = os.path.join(BUILD_DIR, "app.bin")
    firmware_bin = os.path.join(BUILD_DIR, "firmware.bin")
    firmware_elf = os.path.join(BUILD_DIR, "firmware.elf")
    flasher_args_json = os.path.join(BUILD_DIR, "flasher_args.json")

    for required in (bootloader_bin, partitions_bin, firmware_bin, firmware_elf):
        if not os.path.isfile(required):
            print("wokwi_postbuild: missing {}, skipping".format(required))
            return

    # ---- 1. Generate a standalone app.bin from firmware.elf using
    #         esptool elf2image. This is the canonical ESP app image
    #         (0xE9 header + segments + SHA256) and does not require
    #         fabricating or truncating bytes from the merged image.
    if os.path.isfile(app_bin):
        try:
            os.remove(app_bin)
        except OSError:
            pass

    rc = _run([
        "python", ESPTOOL_PY,
        "--chip", "esp32",
        "elf2image",
        "--output", app_bin,
        firmware_elf,
    ])
    if rc != 0 or not os.path.isfile(app_bin):
        print("wokwi_postbuild: elf2image failed, skipping firmware.bin rewrite")
        return

    # ---- 2. Regenerate firmware.bin with the correct flash layout so
    #         the bootloader sits at 0x1000 (with 0xFF padding) and the
    #         app at 0x10000, padded to 2MB.
    rc = _run([
        "python", ESPTOOL_PY,
        "--chip", "esp32",
        "merge_bin",
        "--output", firmware_bin,
        "--flash_mode", "dio",
        "--flash_size", "2MB",
        "--flash_freq", "40m",
        "--fill-flash-size", "2MB",
        "0x1000", bootloader_bin,
        "0x8000", partitions_bin,
        "0x10000", app_bin,
    ])
    if rc != 0:
        print("wokwi_postbuild: merge_bin failed (rc={})".format(rc))
        return

    # ---- 3. Write a corrected flasher_args.json that references the
    #         existing artifacts. Wokwi reads each path relative to
    #         flasher_args.json's directory (.pio/build/esp32dev/) so
    #         bare filenames resolve correctly.
    flasher_args_content = """{
    "write_flash_args" : [ "--flash-mode", "dio",
                           "--flash-size", "2MB",
                           "--flash-freq", "40m" ],
    "flash_settings" : {
        "flash_mode": "dio",
        "flash_size": "2MB",
        "flash_freq": "40m"
    },
    "flash_files" : {
        "0x1000"  : "bootloader.bin",
        "0x8000"  : "partitions.bin",
        "0x10000" : "app.bin"
    },
    "bootloader"      : { "offset" : "0x1000",  "file" : "bootloader.bin", "encrypted" : "false" },
    "partition-table" : { "offset" : "0x8000",  "file" : "partitions.bin", "encrypted" : "false" },
    "app"             : { "offset" : "0x10000", "file" : "app.bin",        "encrypted" : "false" },
    "extra_esptool_args" : {
        "after"  : "hard-reset",
        "before" : "default-reset",
        "stub"   : true,
        "chip"   : "esp32"
    }
}
"""
    with open(flasher_args_json, "w", encoding="utf-8", newline="\n") as f:
        f.write(flasher_args_content)

    print("wokwi_postbuild: wrote {} (bootloader/partitions/app offsets explicit)".format(
        os.path.relpath(flasher_args_json, PROJECT_DIR)))


# Hook into the firmware.bin build target so the rewrite happens AFTER the
# native merge_bin produces the file.
firmware_bin_target = os.path.join(BUILD_DIR, "firmware.bin")
if os.path.isfile(firmware_bin_target):
    env.AddPostAction(firmware_bin_target, _rewrite_firmware_bin)
else:
    env.AddPostAction("$BUILD_DIR/firmware.bin", _rewrite_firmware_bin)
