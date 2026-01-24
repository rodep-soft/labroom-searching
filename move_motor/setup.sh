#!/bin/bash

# .venvフォルダがなければ作成
if [ ! -d "./.venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv ./.venv

fi

# 仮想環境を有効化
source ./.venv/bin/activate
# pyserialをインストール
pip install pyserial

pip list | grep pyserial > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Error: pyserial installation failed."
    echo "Please check your Python and pip installation."
    exit 1
fi

echo "--------------------------------"
echo "Setup complete"
echo "If you want to inactivate, run 'deactivate'"