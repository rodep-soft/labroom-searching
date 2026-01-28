#ifndef COMMON_H
#define COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

// 共有メモリで用いる構造体
typedef struct {
    double vel_x;
    double vel_y;
    double angular_z;
} VelocityCommand;

#define SHM_NAME "/vel_command_shm"

#ifdef __cplusplus
}
#endif

#endif