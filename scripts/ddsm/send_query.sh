#!/bin/bash

# このスクリプトはクエリでIDを確認するためのスクリプト
# 帰ってきた数字の羅列の最初の２文字がID
# 0102...なら01がid

# 
# バス上に一つのモーターだけ繋がっていることを確認
# 0xC8はブロードキャスト的なコマンドなので
# 複数台あるとコリジョンする
# USBアダプタのLED(TX/RX)が光らなかったら遅れてない(他のスクリプトでも言えるが)

PORT="/dev/ttyUSB0"
BAUD="115200"

# port config
stty -F $PORT $BAUD raw -echo cs8 -cstopb -parenb

echo "Querying..."

# IDクエリコマンドを送信し、直後に返信を読み取る
(printf '\xC8\x64\x00\x00\x00\x00\x00\x00\x00\xDE' > $PORT) & 
timeout 0.5s xxd -l 10 -p < $PORT


