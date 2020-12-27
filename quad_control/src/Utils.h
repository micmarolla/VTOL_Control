#ifndef _QUAD_CTRL_UTILS
#define _QUAD_CTRL_UTILS

#include <Eigen/Dense>

using namespace Eigen;

// Compute skew-symmetric matrix of vector v
Matrix3d skew(Vector3d v){
    Matrix3d s;
    s <<    0,      -v[2],  v[1],
            v[2],   0,      -v[0],
            -v[1],  v[0],   0;
    return s;
}

#endif
