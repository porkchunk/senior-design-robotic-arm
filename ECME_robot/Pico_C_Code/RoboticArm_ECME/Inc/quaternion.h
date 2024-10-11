#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "robot_commands.h"

typedef struct {
    float w, x, y, z;
} Quaternion;

Quaternion rotation_matrix_to_quaternion(float R[3][3]);

#endif