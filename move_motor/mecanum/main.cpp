#include <stdio.h>
#include <time.h>
#include "common.h"

// 構造体の初期化
VelocityCommand cmd = {0.0, 0.0, 0.0};

extern "C" {
    int init_dual();
    void update_dual(int fd);
    int init_ddsm();
    void update_ddsm(int fd);
}

int main() {
    int fd_dual = init_dual();
    int fd_ddsm = init_ddsm();

    if (fd_dual < 0 || fd_ddsm < 0) return 1;

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    printf("Robot System Started. 50Hz Loop Running...\n");

    while (1) {
        // 次の時刻（20ms後）を設定
        next_time.tv_nsec += 20000000;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec += 1;
            next_time.tv_nsec -= 1000000000;
        }

        update_dual(fd_dual);
        update_ddsm(fd_ddsm);

        // 正確に待機
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    return 0;
}