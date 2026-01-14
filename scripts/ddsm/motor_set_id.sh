#!/bin/bash

# このスクリプトは16進数でモータのidを引数にとり指定し、
# 一つのバスに繋がったモーターのidを変更する。
# モードの変更や実際にモーターを回すことはしない。

# 以下公式のデータシートから
#
# [Steps]
# HERE -> 1.Set the motor ID (save when power is off)
# 2.Set the motor mode (current loop, speed loop, position loop, the default is speed loop)
# 3.Send the given value
#
# [Note]
# When setting the ID, please ensure that 
# there is only one motor on the bus. 
# It is only allowed to be set once 
# each time the power is turned on. 
# The motor can be set after receiving 5 ID setting instructions.

# [Configurations]
# BaudRate 115200
# Databit 8
# Stopbit 1
# No parity

# ポートは権限に注意
PORT="/dev/ttyUSB0"

BAUD="115200"

# 引数チェック
if [ -z "$1" ]; then
  echo "Usage: $0 <id>"
  exit 1
fi

# 割り振りたいID
# 16進数で指定すると安全
ID_HEX=$1

# デバイスの存在チェック
if [ ! -e "$PORT" ]; then
  # デバイスが認識してないか、名前が違う
  echo "Error: Device $PORT not found."
  exit 1
fi


# シリアルポート設定
stty -F $PORT $BAUD cs8 -cstopb -parenb

# 5回連続で送信しなければならない
# コマンド送信後、一度モーターの電源を切る必要がある
for i in {1..5}
do
  printf "\xAA\x55\x53\x${ID_HEX}\x00\x00\x00\x00\x00\x00" > $PORT
  sleep 0.05
done

echo "Command sent:"
