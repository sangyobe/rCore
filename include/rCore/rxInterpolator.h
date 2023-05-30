/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */

/**
@file rxInterpolator.h
@author Jonghoon Park
@date 2008/9/1
*/
#ifndef __RXINTERPOLATOR_H__
#define __RXINTERPOLATOR_H__

#include <rCore/Interpolator.h>

namespace rlab {
namespace rxcontrolsdk {

//!
/*! represent interpolator type. */
enum eInterpolatorType {
  eInterpolatorType_None = 0,   /*!< */
  eInterpolatorType_Linear,     /*!< */
  eInterpolatorType_Quadratic,  /*!< */
  eInterpolatorType_Cubic,      /*!< */
  eInterpolatorType_Quintic,    /*!< */
  eInterpolatorType_Jerk,       /*!< */
  eInterpolatorType_UserDefined /*!< */
};

/**
 * rxInterpolator class.
 * @brief class that interpolates a scalar variable using polynomial
 */
class rxInterpolator {
public:
  /**
   * construct the rxInterpolator using boundary conditions
   * @param type interpolator type of eInterpolatorType
   * @param t0 start time
   * @param tf final time
   * @param initial initial boundary condition of dim. 3, i.e. (pos, vel, acc)
   * @param final final boundary condition of dim. 3, i.e. (pos, vel, acc)
   */
  rxInterpolator(int type, double t0, double tf, const rMath::dVector &initial,
                 const rMath::dVector &final);
  rxInterpolator(const rxInterpolator &rhs);
  ~rxInterpolator();

  /**
   * reconfigure the interpolator using new boundary conditions
   * @param type interpolator type of eInterpolatorType
   * @param t0 start time
   * @param tf final time
   * @param initial initial boundary condition of dim. 3, i.e. (pos, vel, acc)
   * @param final final boundary condition of dim. 3, i.e. (pos, vel, acc)
   */
  void reconfigure(int type, double t0, double tf,
                   const rMath::dVector &initial, const rMath::dVector &final);

  /**
   * interpolate the desired position, velocity, and acceleration at time t
   * @param t time
   * @param p desired position
   * @param v desired velocity
   * @param a desired acceleration
   */
  void interpolate(double t, double &p, double &v, double &a) const;

private:
  rCore::Interpolator *_impl;
};

} // namespace rxcontrolsdk
} // namespace rlab

#endif
