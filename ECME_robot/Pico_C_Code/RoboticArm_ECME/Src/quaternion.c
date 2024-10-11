#include "quaternion.h"

Quaternion rotation_matrix_to_quaternion(float R[3][3]) {
    Quaternion quaternion;
    float trace = R[0][0] + R[1][1] + R[2][2];

    if (trace > 0.0f) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        quaternion.w = 0.25f / s;
        quaternion.x = (R[1][2] - R[2][1]) * s;
        quaternion.y = (R[2][0] - R[0][2]) * s;
        quaternion.z = (R[0][1] - R[1][0]) * s;
    } 

    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]){
        float s = 2.0f * sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]);
        quaternion.w = (R[1][2] - R[2][1]) / s;
        quaternion.x = 0.25f * s;
        quaternion.y = (R[0][1] + R[1][0]) / s;
        quaternion.z = (R[0][2] + R[2][0]) / s;
    } 

    else if (R[1][1] > R[2][2]) {
        float s = 2.0f * sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]);
        quaternion.w = (R[2][0] - R[0][2]) / s;
        quaternion.x = (R[0][1] + R[1][0]) / s;
        quaternion.y = 0.25f * s;
        quaternion.z = (R[1][2] + R[2][1]) / s;
    } 

    else {
        float s = 2.0f * sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]);
        quaternion.w = (R[0][1] - R[1][0]) / s;
        quaternion.x = (R[0][2] + R[2][0]) / s;
        quaternion.y = (R[1][2] + R[2][1]) / s;
        quaternion.z = 0.25f * s;
    }

    return quaternion;
}

