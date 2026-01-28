#ifndef COMMON_H
#define COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

// 共有する構造体
typedef struct {
    double vel_x;
    double vel_y;
    double angular_z;
} VelocityCommand;

// 変数の実体は main.cpp にあることを宣言
extern VelocityCommand cmd;

#ifdef __cplusplus
}
#endif

#endif