# RTK Base Station - Quickstart (This Machine)

Quick setup for moving this project to a new directory on this machine.

## Prerequisites Already Met ✅

- ESP-IDF v5.5.0 installed at `~/esp/v5.5/esp-idf`
- Python dependencies updated
- Tools configured

## Move Project & Run

```bash
# 1. Move project to new location
mv rtk_base_station /path/to/new/location/
cd /path/to/new/location/rtk_base_station

# 2. Set ESP-IDF environment
source ~/esp/v5.5/esp-idf/export.sh

# 3. Clean and build
idf.py fullclean
idf.py build

# 4. Run in QEMU
idf.py qemu monitor
```

## Expected Output

```
I (140) RTK_BASE: === Minimal PPL RTK Base Station ===
I (140) RTK_BASE: Target: ESP32-S3 QEMU
I (140) RTK_BASE: Point_Perfect_SDK-ESP32-BLD21-v1.11.4
I (140) RTK_BASE: PPL_Initialize: 0
I (140) RTK_BASE: PPL_SendDynamicKey: 0
I (140) RTK_BASE: === PPL Ready ===
```

## Usage

Send data via QEMU console:
- **NMEA**: `$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47`
- **RTCM**: `!R <length>\n<binary_data>`
- **SPARTN**: `!S <length>\n<binary_data>`

Exit: `Ctrl+]`

## One-Liner for Quick Test

```bash
cd /new/path && source ~/esp/v5.5/esp-idf/export.sh && idf.py fullclean build qemu monitor
```

---
*ESP-IDF v5.5.0 | ESP32-S3 QEMU | 150-line minimal implementation*


//running the emulator 
bash -lc 'source ~/esp/v5.5/esp-idf/export.sh && cd /Users/delta/streamrtcm/rtkesp32s3 && qemu-system-xtensa -M esp32s3 -drive file=build/qemu_flash.bin,if=mtd,format=raw -drive file=build/qemu_efuse.bin,if=none,format=raw,id=efuse -global driver=nvram.esp32c3.efuse,property=drive,value=efuse -global driver=timer.esp32s3.timg,property=wdt_disable,value=true -nic user,model=open_eth -nographic -serial tcp:localhost:5555,server,nowait'

//running the feeder 
python3 /Users/delta/streamrtcm/client_serial_feeder.py --serial socket://localhost:5555