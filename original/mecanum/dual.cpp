#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <stdlib.h>
#include "common.h"

#define DEADZONE 0.1

extern "C" int init_dual() {
    int fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (fd == -1) perror("DualSense open error");
    return fd;
}

extern "C" void update_dual(int fd) {
    if (fd < 0) return;
    struct js_event e;
    static double l2_value = 0.0;
    static double r2_value = 0.0;

    auto apply_deadzone = [](double value) {
        if (value > -DEADZONE && value < DEADZONE) return 0.0;
        return value;
    };

    // 全ての溜まっているイベントを処理する
    while (read(fd, &e, sizeof(e)) > 0) {
        unsigned char event_type = e.type & ~JS_EVENT_INIT;
        if (event_type == JS_EVENT_AXIS) {
            double val = 0.0;

            if(e.number == 2) {

                // [-1, 0]に変換し左旋回
                double raw = (double)e.value / 32767.0;
                l2_value = apply_deadzone(-(raw + 1.0) / 2.0);
                cmd.angular_z = r2_value + l2_value;

            } else if(e.number == 5) {

                // [0, 1]に変換し右旋回
                double raw = (double)e.value / 32767.0;
                r2_value = apply_deadzone((raw + 1.0) / 2.0);
                cmd.angular_z = r2_value + l2_value;

            } else {
                // スティックの値を -1.0 ~ 1.0 に正規化
                val = (double)e.value / 32767.0;
                val = apply_deadzone(val);
            }

            // 番号は js_test 等で確認した DualSense の標準的な割当
            switch (e.number) {
                case 1: // 前進・後進 <- Ly
                    cmd.vel_x = -val; // 上がマイナスなので反転
                    break;
                case 3: // 左右スライド <- Ry
                    cmd.vel_y = val;
                    break;
                case 2: // L2
                case 5: // R2
                    cmd.angular_z = apply_deadzone(cmd.angular_z);
                    break;
                default:
                    break;
            }
        }
    }
}