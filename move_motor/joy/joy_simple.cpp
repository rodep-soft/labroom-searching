#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <stdbool.h>

int main(void) {
    // ノンブロッキングモードでジョイスティックデバイスを開く
    const char *device = "/dev/input/js0";
    int fd = open(device, O_RDONLY | O_NONBLOCK);

    if (fd == -1) {
        perror("デバイスを開けませんでした");
        return 1;
    }

    // ドライバのバージョンを取得
    int version;
    if (ioctl(fd, JSIOCGVERSION, &version) < 0) {
        fprintf(stderr, "1.0+ API (event interface) に対応していません。\n");
        close(fd);
        return 1;
    }
    printf("Driver version: %d.%d.%d\n",
           version >> 16, (version >> 8) & 0xff, version & 0xff);

    // ジョイスティックの名前を取得
    char name[128];
    if (ioctl(fd, JSIOCGNAME(sizeof(name)), name) < 0) {
        printf("名前を取得できませんでした。\n");
    } else {
        printf("Joystick Name: %s\n", name);
    }

    printf("--- 読み取り開始 (Ctrl+C で終了) ---\n");

    // ジョイスティックの状態を読み取る
    struct js_event e;
    while (1) {
        ssize_t bytes = read(fd, &e, sizeof(e));

        // イベントが読み取れた場合に表示
        if (bytes > 0) {
            bool is_init = (e.type & JS_EVENT_INIT);
            const char *type_str = (e.type & JS_EVENT_BUTTON) ? "Button" : "Axis";

            // イベント情報を表示
            printf("[%s] 番号: %d, 値: %d %s\n",
                   type_str, e.number, e.value, is_init ? "(初期化データ)" : "");
        }

        usleep(10000);
    }

    close(fd);
    return 0;
}