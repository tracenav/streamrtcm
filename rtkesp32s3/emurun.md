# RTK Base Station - QEMU Testing

ESP32-S3 PointPerfect library testing with NTRIP SPARTN streams in QEMU.

## Build

```bash
cd /Users/delta/streamrtcm/rtkesp32s3
source ~/esp/v5.5/esp-idf/export.sh
idf.py fullclean build qemu  # qemu generates flash images
```

**Note:** `idf.py qemu` creates `build/qemu_flash.bin` and `build/qemu_efuse.bin` required for QEMU.

## Run Test

### Check for Existing QEMU Processes

Before starting, check if QEMU is already running:

```bash
# Check for running QEMU processes
ps aux | grep qemu-system-xtensa | grep -v grep

# Check if port 5555 is in use
lsof -i :5555
```

If QEMU is already running, kill it first:

```bash
pkill -9 qemu-system-xtensa
```

### Terminal 1: Start QEMU Emulator (Background)

```bash
cd /Users/delta/streamrtcm/rtkesp32s3
source ~/esp/v5.5/esp-idf/export.sh
qemu-system-xtensa -M esp32s3 \
  -drive file=build/qemu_flash.bin,if=mtd,format=raw \
  -drive file=build/qemu_efuse.bin,if=none,format=raw,id=efuse \
  -global driver=nvram.esp32c3.efuse,property=drive,value=efuse \
  -global driver=timer.esp32s3.timg,property=wdt_disable,value=true \
  -nic user,model=open_eth -nographic \
  -serial tcp:localhost:5555,server,nowait > qemu.log 2>&1 &

# Verify QEMU started:
ps aux | grep qemu-system-xtensa | grep -v grep

# Watch output (ESP32 firmware output may not appear until serial connection):
tail -f qemu.log
```

**Note:** The `nowait` option allows QEMU to run without waiting for a connection, freeing port 5555 for the feeder.

**Expected output:**
```
PPL QEMU bare passthrough
Point_Perfect_SDK-ESP32-BLD21-v1.11.4
PPL_Initialize: 0 (Success)
PPL ready for IP channel (NTRIP) data
STATUS: ppl=Success key=unset spartn=0/0 rtcm_in=0/0 nmea(GGA/ZDA)=0/0 ppl_out=0/0
```

### Terminal 2: Feed RTCM + SPARTN Data

```bash
python3 /Users/delta/streamrtcm/client_serial_feeder.py \
  --serial socket://localhost:5555 \
  --eph-file /Users/delta/streamrtcm/spartn_logs/rtcmbase_20251021_221928.log \
  --spartn-file /Users/delta/streamrtcm/spartn_logs/spartn_20251021_181915.log \
  --eph-rate 0.05 \
  --gga-lat 38.3032 \
  --gga-lon -77.4605 \
  --gga-alt 30.0
```

**Expected output:**
```
Opened serial socket://localhost:5555 @ 115200
Loaded 188 ephemeris frames from file
Loaded 231 SPARTN frames from file
[ESP32] pushed R 67 bytes (file)
[ESP32] EPH initial batch complete; releasing GGA/SPARTN
[ESP32] NMEA sent: GGA+ZDA (1021, 22:19:16 UTC, 38.30320, -77.46050)
[ESP32] pushed S 41 bytes (spartn-file)
[ESP32<-] RTK OUT: 528 bytes | types: 1005,1033,1230,1074,1084,1094
[ESP32<-] STATUS: ppl=Success spartn=101/5160 rtcm_in=376/23912 ppl_out=3/1584
```

## What It Does

1. **Loads data from logs:**
   - RTCM ephemeris (1019/1020/1042/1044/1045/1046) at 20 msg/sec
   - SPARTN corrections (OCB/HPAC) from NTRIP stream

2. **Time sync:**
   - GGA/ZDA use log timestamps (not system time)
   - Starts at log generation time: 22:19:15 UTC

3. **PPL output:**
   - Station position (1005)
   - Antenna descriptor (1033)
   - GLONASS bias (1230)
   - MSM4 observations (1074 GPS, 1084 GLO, 1094 GAL)
   - Updates every ~5 seconds

## Configuration

**GGA Position** (Fredericksburg, VA):
- Latitude: 38.3032°N
- Longitude: -77.4605°W
- Altitude: 30.0m

**Feeder Options:**
- `--eph-rate 0.05` - 20 RTCM msg/sec (default: 0.1 = 10 msg/sec)
- `--no-zda` - Disable ZDA time messages (GGA only)
- `--speed 2.0` - 2x replay speed

## PPL Configuration

**IP Channel** (NTRIP SPARTN streams):
- `PPL_Initialize(PPL_CFG_ENABLE_IP_CHANNEL)` 
- `PPL_SendSpartn()` for corrections
- Unencrypted stream (no dynamic key needed)

**Not used:** AUX channel is for L-band PMP streams only.

## Kill QEMU

```bash
pkill -9 qemu-system-xtensa
# or
lsof -ti:5555 | xargs kill -9
```

---

*ESP-IDF v5.5.0 | ESP32-S3 QEMU | PointPerfect IP channel*
