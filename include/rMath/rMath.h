/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
// Interpolator.h: interface for the Interpolation class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __RMATH_RMATH_H__
#define __RMATH_RMATH_H__

#define EIGEN_MATRIXBASE_PLUGIN "rMath/EigenMatrixBasePlugin.h"
#define EIGEN_MATRIX_PLUGIN "rMath/EigenMatrixPlugin.h"
#define EIGEN_TRANSFORM_PLUGIN "rMath/EigenTransformPlugin.h"
#include <Eigen/Eigen>

namespace rMath {
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;
typedef Eigen::Matrix<double, 2, 2> Matrix2d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
} // namespace rMath

#endif // __RMATH_RMATH_H__
