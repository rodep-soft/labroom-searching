import serial
import sys
import time

# crc計算
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

def send_vel_command(port, m_rpm):

    # rpm制限
    rpm = max(min(m_rpm,330), -330)

    ser = None
    try:
        # シリアルポートを開く
        ser = serial.Serial(port, 115200, timeout = 1)

        # motor IDの1~4までのpacketを作成して送信
        for i in range(1, 5):
            motor_id = i
            # モーターが逆回転のため-をかける
            if i in [1, 3]:
                send_rpm = -rpm
            else:
                send_rpm = rpm

            # int -> signed16
            rpm_16 = format(send_rpm & 0xFFFF, '016b')

            # packet作成
            packet = bytearray([
                i,                  # DARA[0] - Motor ID
                0x64,               # DATA[1]
                int(rpm_16[0:8],2), # DATA[2] - 16進数: {hex(int(rpm_16[0:8],2))}
                int(rpm_16[8:16],2),# DATA[3] - 16進数: {hex(int(rpm_16[8:16],2))}
                0x00,               # DATA[4]
                0x00,               # DATA[5]
                0x00,               # DATA[6]
                0x00,               # DATA[7]
                0x00                # DATA[8]
            ])

            crc = crc8_maxim(packet)
            packet.append(crc)  # DATA[9]

            # パケットを送信
            ser.write(packet)
            print(f"Sent to motor {i}: {rpm} rpm")

            # responseを受信
            response = ser.read(10)
            if response:
                print(f"Response from motor {i}: {response.hex(' ').upper()}")
        
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Closing serial port...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser is not None:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("引数が足りません")
        sys.exit(1)

    m_rpm = int(sys.argv[1])
    send_vel_command('/dev/ttyACM0',m_rpm)
