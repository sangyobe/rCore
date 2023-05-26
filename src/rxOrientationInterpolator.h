/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
*/

/** 
@file rxOrientationInterpolator.h
@author Jonghoon Park
@date 2008/9/1
*/
#ifndef __RXORIENTATIONINTERPOLATOR_H__
#define __RXORIENTATIONINTERPOLATOR_H__

#include <rxcontrolsdk/link_pragmas.h>
#include <rlab/math/rMath.h>
#include <rcore/OrientationInterpolator.h>

namespace rlab {
namespace rxcontrolsdk {

/**
* rxOrientationInterpolator class. 
* @brief class that interpolates two rotations using polynomial
*/
class RXCONTROLSDK_IMPEXP rxOrientationInterpolator
{
public:
	/**
	* construct the rxOrientationInterpolator using boundary conditions
	* @param type interpolator type of eInterpolatorType
	* @param t0 start time
	* @param tf final time
	* @param R0 initial rotation at t0
	* @param Rf final rotation at tf
	* @param omega0 initial body angular velocity at t0
	* @param omegaf final body angular velocity at tf
	* @param alpha0 initial body angular acceleration at t0
	* @param alphaf final body angular acceleration at tf
	*/
	rxOrientationInterpolator(int type,
							double t0,
							double tf,
							const Rotation& R0,
							const Rotation& Rf,
							const Vector3D& omega0,
							const Vector3D& omegaf,
							const Vector3D& alpha0,
							const Vector3D& alphaf);
			
	rxOrientationInterpolator(const rxOrientationInterpolator& rhs);
	~rxOrientationInterpolator();

	/**
	* interpolate the desired rotation, body angular velocity, and acceleration at time t
	* @param t time
	* @param R desired rotation
	* @param omega desired body angular velocity
	* @param alpha desired body angular acceleration
	*/
	void interpolate(double t, Rotation& R, Vector3D& omega, Vector3D& alpha) const;

private:
	rxOrientationInterpolator();
	core::OrientationInterpolator* _impl;

};

} // namespace rxcontrolsdk
} // namespace rlab


#endif
