# mecanumについて
===
- common.hに定義されている構造体VelocityCommandで、ロボットの速度を指定
```bash
- cmd.linear_x: 前後方向の速度
- cmd.linear_y: 左右方向の速度
- cmd.angular_z: 回転方向の速度
```

- dual.cppで、dualsenseをつかって値を決め、ロボットの速度を制御
- ddsm.cで、ロボットの速度を実際に制御する関数を定義


## コンパイル方法
```bash
g++ -o robot main.cpp dual.cpp ddsm.c -lm -Wall
```

## todo
- ラズパイが壊れたらしいのでudevルールを作成する
	- 参考: [udevルール作成記事](https://zenn.dev/karaage0703/articles/d6759ea297dbf8)
- 逆運動学の計算ミスをしていないか
- モーターの周り型の再確認
- 一回やってみてずれるようだったらrox_tempのようにmotor_correctionを入れてみる