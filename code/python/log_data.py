import serial
import re
import csv

PORT = 'COM5'
BAUDRATE = 115200
SAMPLES = 10000
CSV_FILE = 'dac_log.csv'

def main():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser, open(CSV_FILE, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x_byte0', 'x_byte1', 'y_byte0', 'y_byte1', 'dac1', 'dac2'])
        count = 0
        print(f"Logging {SAMPLES} samples to {CSV_FILE}...")
        while count < SAMPLES:
            line = ser.readline().decode(errors='ignore').strip()
            if not line or 'DAC1' not in line:
                continue
            try:
                hex_part = line.split('DAC1')[0].strip()
                dac_part = line.split('DAC1')[1]
                byte_strs = hex_part.split()
                if len(byte_strs) < 33:
                    continue
                data = [int(b, 16) for b in byte_strs]
                dac_match = re.search(r'GPIO25\):\s*(\d+),.*GPIO26\):\s*(\d+)', dac_part)
                if not dac_match:
                    continue
                dac1 = int(dac_match.group(1))
                dac2 = int(dac_match.group(2))
                x0, x1 = data[21], data[22]
                y0, y1 = data[23], data[24]
                writer.writerow([x0, x1, y0, y1, dac1, dac2])
                count += 1
                if count % 50 == 0:
                    print(f"  {count} samples logged...")
            except Exception as e:
                print("Parse error:", e)
        print("Done.")

if __name__ == "__main__":
    main()