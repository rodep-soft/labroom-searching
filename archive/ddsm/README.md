# archive/ddsm の内容

このディレクトリは、DDSM モータ制御まわりの旧コードを退避している場所です。

## ファイル一覧

- `common.h`  
	共有する速度コマンド構造体 `VelocityCommand`（`vel_x`, `vel_y`, `angular_z`）の定義。

- `dual.cpp`  
	`/dev/input/js0`（DualSense）から入力を読み、速度指令（共有データ）を更新する想定のコード。

- `motor_driver.c`  
	速度指令からメカナム逆運動学で各輪の RPM を計算し、`/dev/ddsm` にシリアル送信する制御ループ。

- `same_vector.c`  
	指定 RPM を4輪へ同一ベクトルで送信して確認するためのテスト用コード。


