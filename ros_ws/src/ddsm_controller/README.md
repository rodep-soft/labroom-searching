# ddsm_controllerとkeyboard_teleopの使い方
(まだやったことない)

1. ddsm_controllerの起動
```ros2 run ddsm_controller ddsm_controller_node```
2. keyboard_teleopの起動
```ros2 run teleop_twist_keyboard teleop_twist_keyboard```
3. keyboard_teleopのコマンドを入力してDDSMを操作する
```bash
- a: 前進1.0m/s
- s: 前進1.5m/s
- d: 前進2.0m/s
- f: 後進1.0m/s
- g: 後進1.5m/s
- h: 後進2.0m/s
```