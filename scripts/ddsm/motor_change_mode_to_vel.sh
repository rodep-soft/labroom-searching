#!/bin/bash

# このスクリプトはモータのIDを指定し、
# そのモーターを速度モードに変更するスクリプトである
# IDの設定はすでに済んでいることを前提とする
# 引数に16進数でモータのidを取る
#
# [注意]
# モーターを停止させ、物理的に回転が10rpmになるまで待ってからこのスクリプトを実行すること

# 以下公式のデータシートから
#
# [Steps]
# 1.Set the motor ID (save when power is off)
# HERE-> 2.Set the motor mode (current loop, speed loop, position loop, the default is speed loop)
# 3.Send the given value
#
# [Description]
# Mode value:
# 0x01：set the current loop
# 0x02：set the velocity loop
# 0x03：set the position loop
# The rotating velocity of the motor must be lower than 10rpm 
# when switching to the position loop.

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

# 速度モードに変更したいモータのID
# 16進数で指定すると安全
# 0xはつけなくていい
ID_HEX=$1

# Vel Mode
VEL_MODE="02"

# デバイスの存在チェック
if [ ! -e "$PORT" ]; then
  # デバイスが認識してないか、名前が違う
  echo "Error: Device $PORT not found."
  exit 1
fi


# シリアルポート設定
stty -F $PORT $BAUD cs8 -cstopb -parenb

# 0x02 -> vel loop
# モードを変えたいときは変数を変更する
printf "\x${ID_HEX}\xA0\x00\x00\x00\x00\x00\x00\x00\x${VEL_MODE}" > $PORT

echo "Command sent:"

