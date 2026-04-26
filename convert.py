"""
Y2S2 Launch Day Data Converter
Reads calibData.bin + dataLog.bin from SD card and outputs a CSV.

Usage:
    python convert.py                              # uses calibData.bin / dataLog.bin in same folder
    python convert.py calib.bin log.bin out.csv    # explicit paths
"""

import struct
import csv
import sys
import os

# ---------------------------------------------------------------------------
# Binary formats (little-endian, no padding -- AVR GCC ABI aligns to 1 byte)
# ---------------------------------------------------------------------------

# calibData_t: 33 bytes
# uint16 dig_T1, int16 dig_T2/T3,
# uint16 dig_P1, int16 dig_P2..P9,
# uint8 dig_H1, int16 dig_H2, uint8 dig_H3, int16 dig_H4/H5, int8 dig_H6
CALIB_FMT  = '<HhhHhhhhhhhhBhBhhb'
CALIB_SIZE = struct.calcsize(CALIB_FMT)   # 33 bytes

CALIB_FIELDS = [
    'dig_T1','dig_T2','dig_T3',
    'dig_P1','dig_P2','dig_P3','dig_P4','dig_P5','dig_P6','dig_P7','dig_P8','dig_P9',
    'dig_H1','dig_H2','dig_H3','dig_H4','dig_H5','dig_H6',
]

# dataLog_t (__attribute__((packed))): 32 bytes
# uint32 timestamp, int16 acc_x/y/z, int16 gyro_x/y/z,
# uint32 temp, uint32 pressure, uint32 humidity, uint32 null
LOG_FMT  = '<LhhhhhhLLLL'
LOG_SIZE = struct.calcsize(LOG_FMT)       # 32 bytes

# ---------------------------------------------------------------------------
# BMI270 conversion (configured: ±16g, ±2000 dps)
# ---------------------------------------------------------------------------
ACC_SENS  = 16.0   / 32768.0   # g per LSB
GYRO_SENS = 2000.0 / 32768.0   # dps per LSB

# ---------------------------------------------------------------------------
# BME280 compensation (floating-point, from datasheet)
# Raw temp/pressure are (MSB<<8)|LSB = upper 16 bits of 20-bit ADC
# so shift left 4 to reconstruct the 20-bit adc value before compensation.
# Raw humidity is the full 16-bit adc value (no shift needed).
# ---------------------------------------------------------------------------

def compensate_temp(raw, c):
    adc = raw << 4
    var1 = (adc / 16384.0 - c['dig_T1'] / 1024.0) * c['dig_T2']
    var2 = (adc / 131072.0 - c['dig_T1'] / 8192.0) ** 2 * c['dig_T3']
    t_fine = var1 + var2
    return t_fine / 5120.0, t_fine

def compensate_pressure(raw, t_fine, c):
    adc = raw << 4
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * c['dig_P6'] / 32768.0
    var2 = var2 + var1 * c['dig_P5'] * 2.0
    var2 = var2 / 4.0 + c['dig_P4'] * 65536.0
    var1 = (c['dig_P3'] * var1 * var1 / 524288.0 + c['dig_P2'] * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * c['dig_P1']
    if var1 == 0.0:
        return 0.0
    p = 1048576.0 - adc
    p = (p - var2 / 4096.0) * 6250.0 / var1
    var1 = c['dig_P9'] * p * p / 2147483648.0
    var2 = p * c['dig_P8'] / 32768.0
    p = p + (var1 + var2 + c['dig_P7']) / 16.0
    return p / 100.0  # hPa

def compensate_humidity(raw, t_fine, c):
    x = t_fine - 76800.0
    if x == 0.0:
        return 0.0
    x = (raw - (c['dig_H4'] * 64.0 + c['dig_H5'] / 16384.0 * x)) * \
        (c['dig_H2'] / 65536.0 * (1.0 + c['dig_H6'] / 67108864.0 * x *
        (1.0 + c['dig_H3'] / 67108864.0 * x)))
    x = x * (1.0 - c['dig_H1'] * x / 524288.0)
    return max(0.0, min(100.0, x))

def pressure_to_altitude(p_hpa, p0_hpa):
    """Relative altitude above launch site (m) using barometric formula."""
    return 44330.0 * (1.0 - (p_hpa / p0_hpa) ** (1.0 / 5.255))

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def read_calib(path):
    with open(path, 'rb') as f:
        raw = f.read(CALIB_SIZE)
    if len(raw) < CALIB_SIZE:
        raise ValueError(f"calibData.bin too small: got {len(raw)}, expected {CALIB_SIZE}")
    return dict(zip(CALIB_FIELDS, struct.unpack(CALIB_FMT, raw)))

def read_log(path):
    records = []
    with open(path, 'rb') as f:
        while True:
            chunk = f.read(LOG_SIZE)
            if len(chunk) < LOG_SIZE:
                break
            records.append(struct.unpack(LOG_FMT, chunk))
    return records

def convert(calib_path, log_path, out_path):
    print(f"Reading calibration: {calib_path}")
    calib = read_calib(calib_path)

    print(f"  dig_T1={calib['dig_T1']}  dig_T2={calib['dig_T2']}  dig_T3={calib['dig_T3']}")
    print(f"  dig_P1={calib['dig_P1']}  dig_P2={calib['dig_P2']}")

    print(f"Reading log:         {log_path}")
    raw_records = read_log(log_path)
    if not raw_records:
        print("No records found in dataLog.bin")
        return

    # Use first pressure reading as ground-level reference for altitude
    _, _, _, _, _, _, _, raw_temp0, raw_press0, _, _ = raw_records[0]
    _, t_fine0 = compensate_temp(raw_temp0, calib)
    p0 = compensate_pressure(raw_press0, t_fine0, calib)
    if p0 <= 0:
        p0 = 1013.25  # fall back to ISA standard if compensation fails

    rows = []
    for i, rec in enumerate(raw_records):
        ts, ax, ay, az, gx, gy, gz, raw_temp, raw_press, raw_hum, _ = rec
        if i < 3:
            print(f"  rec[{i}]: raw_temp=0x{raw_temp:08X} raw_press=0x{raw_press:08X} raw_hum=0x{raw_hum:08X}")

        temp_c,  t_fine = compensate_temp(raw_temp, calib)
        press_hpa        = compensate_pressure(raw_press, t_fine, calib)
        hum_pct          = compensate_humidity(raw_hum, t_fine, calib)
        alt_m            = pressure_to_altitude(press_hpa, p0) if press_hpa > 0 else 0.0

        rows.append({
            'timestamp_ms':  ts,
            'acc_x_g':       round(ax * ACC_SENS,  4),
            'acc_y_g':       round(ay * ACC_SENS,  4),
            'acc_z_g':       round(az * ACC_SENS,  4),
            'gyro_x_dps':    round(gx * GYRO_SENS, 3),
            'gyro_y_dps':    round(gy * GYRO_SENS, 3),
            'gyro_z_dps':    round(gz * GYRO_SENS, 3),
            'temp_c':        round(temp_c,   2),
            'press_hpa':     round(press_hpa, 2),
            'altitude_m':    round(alt_m,    2),
            'humidity_pct':  round(hum_pct,  2),
        })

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print(f"Converted {len(rows)} records -> {out_path}")
    print(f"  Ground pressure:  {p0:.2f} hPa")
    print(f"  Peak altitude:    {max(r['altitude_m'] for r in rows):.1f} m")
    print(f"  Max accel (Y):    {max(abs(r['acc_y_g']) for r in rows):.2f} g")
    print(f"  Duration:         {(rows[-1]['timestamp_ms'] - rows[0]['timestamp_ms'])/1000:.1f} s")


if __name__ == '__main__':
    base       = os.path.dirname(os.path.abspath(__file__))
    calib_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(base, 'calibData.bin')
    log_path   = sys.argv[2] if len(sys.argv) > 2 else os.path.join(base, 'dataLog.bin')
    out_path   = sys.argv[3] if len(sys.argv) > 3 else os.path.join(base, 'output.csv')
    convert(calib_path, log_path, out_path)
