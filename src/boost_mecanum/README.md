# boost_mecanum

このディレクトリには、Boost.Asio を使ってシリアル通信で 4 輪メカナムモーターへ同一 RPM を送信する C++ コードがあります。

- 実装内容:
  - 引数で `rpm` を受け取る
  - モーター ID 1〜4 に同じ RPM を順番に送信
  - RPM を `-330 ~ 330` に制限
  - ID 1,3 は回転方向を反転
  - CRC-8/MAXIM を計算してパケット末尾に付与
  - 送信後に受信データがあれば 16 進表示
- シリアル設定:
  - ポート: `/dev/ddsm`
  - ボーレート: `115200`
  - データ長: 8bit
  - パリティ: なし
  - ストップビット: 1
  - フロー制御: なし

## コンパイル

プロジェクトルートで実行:

```bash
g++ -std=c++17 mv_same.cpp -o mv_same -lboost_system -lpthread
```

## 実行

```bash
./mv_same <rpm>
```

例:

```bash
./mv_same 30
```