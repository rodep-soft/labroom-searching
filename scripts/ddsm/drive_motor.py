import serial
import sys

# CRC計算
def crc8_maxim(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C # Polynomial inverse of 0x31
            else:
                crc >>= 1
    return crc

def send_velocity_command(port, motor_id, rpm):
    # In speed loop mode: -330~330, unit rpm, data type signed 16-bit
    rpm = max(min(rpm, 330), -330)
    
    # Intからsigned int16
    # ビッグエンディアン
    # プロトコル: DATA[2]=High, DATA[3]=Low
    speed_bytes = rpm.to_bytes(2, byteorder='big', signed=True)
    
    # パケット組み立て (DATA[0]~DATA[8])
    # ID, 0x64, SpeedH, SpeedL, 0, 0, 0, 0, 0
    packet = bytearray([
        motor_id,       # DATA[0]
        0x64,           # DATA[1]
        speed_bytes[0], # DATA[2] High
        speed_bytes[1], # DATA[3] Low
        0x00,           # DATA[4]
        0x00,           # DATA[5] Accel
        0x00,           # DATA[6] Brake (0x00=No, 0xFF=Yes)
        0x00,           # DATA[7]
        0x00            # DATA[8]
    ])
    
    crc = crc8_maxim(packet)
    packet.append(crc)  # DATA[9]
    
    try:
        with serial.Serial(port, 115200, timeout=0.1) as ser:
            ser.write(packet)
            print(f"Sent to ID {motor_id}: {rpm} rpm")
            print(f"Packet: {packet.hex(' ').upper()}")
            
            response = ser.read(10)
            if response:
                print(f"Response: {response.hex(' ').upper()}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 drive_motor.py <ID_DEC> <RPM_DEC>")
        sys.exit(1)
        
    m_id = int(sys.argv[1])
    m_rpm = int(sys.argv[2])
    send_velocity_command('/dev/ddsm', m_id, m_rpm)
