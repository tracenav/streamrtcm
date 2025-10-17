# RTK Base Station - QEMU Testing

ESP32-S3 PointPerfect library testing with NTRIP SPARTN streams in QEMU.

## Quick Start

```bash


# 1. Build and start QEMU (Terminal 1)
cd /Users/delta/streamrtcm/rtkesp32s3

source ~/esp/v5.5/esp-idf/export.sh

idf.py fullclean build qemu  # qemu generates flash images

# 1b. Kill any existing QEMU if stuck 
pkill -9 qemu-system-xtensa

bash -lc 'source ~/esp/v5.5/esp-idf/export.sh && cd /Users/delta/streamrtcm/rtkesp32s3 && qemu-system-xtensa -M esp32s3 -drive file=build/qemu_flash.bin,if=mtd,format=raw -drive file=build/qemu_efuse.bin,if=none,format=raw,id=efuse -global driver=nvram.esp32c3.efuse,property=drive,value=efuse -global driver=timer.esp32s3.timg,property=wdt_disable,value=true -nic user,model=open_eth -nographic -serial tcp:localhost:5555,server,nowait' 2>&1 | head -50


# 2. Feed data (Terminal 2)
python3 /Users/delta/streamrtcm/client_serial_feeder.py \
  --serial socket://localhost:5555 \
  --eph-file /Users/delta/streamrtcm/example1logs/rtcmbase_20251021_221928.log \
  --spartn-file /Users/delta/streamrtcm/example1logs/spartn_20251021_181915.log \
  --eph-rate 0.05 \
  --gga-lat 38.3032 --gga-lon -77.4605 --gga-alt 30.0
```


