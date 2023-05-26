/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
// Interpolator.h: interface for the Interpolation class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __INTERPOLATOR_H__
#define __INTERPOLATOR_H__

#include "link_pragmas.h"
#define EIGEN_MATRIXBASE_PLUGIN "rCore/EigenMatrixBasePlugin.h"
#define EIGEN_MATRIX_PLUGIN "rCore/EigenMatrixPlugin.h"
// #define	EIGEN_TRANSFORM_PLUGIN "rCore/EigenTransformPlugin.h"
#include <Eigen/Eigen>

namespace rCore {
class RCORE_IMPEXP Interpolator {
public:
  enum TYPE {
    NONE = 0,
    LINEAR = 1,
    QUADRATIC = 2,
    CUBIC = 3,
    QUINTIC = 4,
    JERK = 5,
    // LINEAR_PARABOLIC_BLEND = 6
    // CSPLINE_NATURAL,
    // CSPLINE_CLAMPED,
    // CSPLINE_FORCED
  };

  // 	struct LPBParm
  // 	{
  // 		double v0, vf;
  // 		double p0, pf;
  // 		double ta;		// acceleration duration
  // 		double td;		// deceleration duration
  // 		double vmax;	// maximum velocity
  // 		double amax;	// maximum acceleration
  // 		bool isAscent;	// flag whether the final position is positively
  // greater than the initial position 		LPBParm() : ta(-1), td(-1), vmax(-1),
  // amax(-1), isAscent(true) { }
  // 	};

public:
  typedef Eigen::Matrix<double, -1, 1> VectorXd;
  typedef Eigen::Matrix<double, 2, 1> Vector2d;
  typedef Eigen::Matrix<double, 3, 1> Vector3d;
  typedef Eigen::Matrix<double, 4, 1> Vector4d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, -1, -1> MatrixXd;
  typedef Eigen::Matrix<double, 2, 2> Matrix2d;
  typedef Eigen::Matrix<double, 3, 3> Matrix3d;
  typedef Eigen::Matrix<double, 4, 4> Matrix4d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  Interpolator() : _type(Interpolator::TYPE::NONE), _t0(0), _tf(0), _coeff() {}
  virtual ~Interpolator() {}

  Interpolator(const Interpolator::TYPE type, const double t0, const double tf,
               const Eigen::VectorXd &initial, const Eigen::VectorXd &final);

  void reconfigure(const Interpolator::TYPE type, const double t0,
                   const double tf, const VectorXd &initial,
                   const VectorXd &final);

  void interpolate(const double t, double &p, double &v, double &a) const;

private:
  void _determineCoeff(const VectorXd &initial,
                       const VectorXd &final /*, void* data = NULL*/);

private:
  Interpolator::TYPE _type;
  double _t0;
  double _tf;
  VectorXd _coeff;
};
} // namespace rCore

#endif // __INTERPOLATOR_H__
