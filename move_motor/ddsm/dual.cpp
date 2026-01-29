#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <stdlib.h>

#include "common.h"

#define DEADZONE 0.10

int main() {
    const char *device = "/dev/input/js0";
    // ノンブロッキングモードで開く
    int fd = open(device, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        perror("DualSenseが見つかりません");
        return 1;
    }

    // 構造体の初期化
    cmd->vel_x = 0.0;
    cmd->vel_y = 0.0;
    cmd->angular_z = 0.0;

    printf("--- DualSense 制御開始---\n");

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
                    case 1: // 左スティック 縦 (前進・後進) <- Lx
                        cmd->vel_x = -val; // 上がマイナスなので反転
                        break;
                    case 4: // 左スティック 横 (左右スライド) <- Ry
                        cmd->vel_y = val;
                        break;
                    case 2: // 左スティック 横 (旋回) <- L2
                        cmd->angular_z = val;
                        break;
                    case 5: // 右スティック 縦 (旋回) <- R2
                        cmd->angular_z = -val; // 右スティックでの旋回は反転
                        break;
                    default:
                        break;
                }
            }
        }

        // デバッグ表示（確認用）
        printf("\rTarget Velocity -> X:%.2f, Y:%.2f, Z:%.2f", cmd->vel_x, cmd->vel_y, cmd->vel_z);
        fflush(stdout);

        usleep(10000); // 100Hz周期
    }

    // 終了処理
    close(fd);
    return 0;
}