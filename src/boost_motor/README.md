# boost_motor

このディレクトリのコードは、Boost.Asio を使ってシリアル通信で **1つのモーター** に速度指令を送るためのものです。

## コンパイル

```bash
g++ -std=c++17 motor.cpp -o motor_single -lboost_system -lpthread
```

## 実行

```bash
./motor_single <motor_id> <speed_rpm>
```

例:

```bash
./motor_single 2 120
```