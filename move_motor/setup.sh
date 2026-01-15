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

echo "--------------------------------"
echo "Setup complete"
echo "If you want to inactivate, run 'deactivate'"