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
    while (1) {
        // 全ての溜まっているイベントを処理する
        while (read(fd, &e, sizeof(e)) > 0) {
            if (e.type & JS_EVENT_AXIS) {
                // スティックの値を -1.0 ~ 1.0 に正規化
                double val = 0.0;
                val = (double)e.value / 32767.0;
                
                // デッドゾーン処理
                if (val > -DEADZONE && val < DEADZONE) {
                    val = 0.0;
                }

                // 番号は js_test 等で確認した DualSense の標準的な割当
                switch (e.number) {
                    case 1: // 前進・後進 <- Lx
                        cmd.vel_x = -val; // 上がマイナスなので反転
                        break;
                    case 4: // 左右スライド <- Ry
                        cmd.vel_y = val;
                        break;
                    case 2: // 左旋回 <- L2
                        cmd.angular_z = val;
                        break;
                    case 5: // 右旋回 <- R2
                        cmd.angular_z = -val; // 右スティックでの旋回は反転
                        break;
                    default:
                        break;
                }
            }
        }
    }
}