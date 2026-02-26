#ifndef COMMON_H
#define COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double vel_x;
    double vel_y;
    double angular_z;
} VelocityCommand;

extern VelocityCommand cmd;

#ifdef __cplusplus
}
#endif

#endif