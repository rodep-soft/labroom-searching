# archive/joy の内容

このディレクトリは、ジョイスティック入力確認まわりの旧コードと生成物を退避している場所です。

## ファイル一覧

- `joy_simple.cpp`  
  `/dev/input/js0` のイベントをそのまま表示するシンプルな確認用コード。
  コンパイルコマンドは `g++ joy_simple.cpp -o joy_simple` などで、ビルド済みバイナリも同名で配置

- `joy_echo.cpp`  
  軸・ボタン状態を配列で保持し、`ros2 topic echo` 風に継続表示する確認用コード。
  コンパイルコマンドは `g++ joy_echo.cpp -o joy_echo` などで、ビルド済みバイナリも同名で配置



## メモ

- `simple` と `echo` はソースではなくビルド済みバイナリです。
