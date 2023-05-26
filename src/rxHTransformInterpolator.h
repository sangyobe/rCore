/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
*/

/** 
@file rxHTransformInterpolator.h
@author Jonghoon Park
@date 2008/9/1
*/
#ifndef __RXHTRANSFORMINTERPOLATOR_H__
#define __RXHTRANSFORMINTERPOLATOR_H__

#include <rxcontrolsdk/link_pragmas.h>
#include <rlab/math/rMath.h>
#include <rcore/HTransformInterpolator.h>

namespace rlab {
namespace rxcontrolsdk {

/**
* rxHTransformInterpolator class. 
* @brief class that interpolates two transformations using polynomial
*/
class RXCONTROLSDK_IMPEXP rxHTransformInterpolator 
{
public:
	/**
	* construct the rxHTransformInterpolator using boundary conditions
	* @param type interpolator type of eInterpolatorType
	* @param t0 start time
	* @param tf final time
	* @param T0 initial transformation at t0
	* @param Tf final transformation at tf
	* @param V0 initial body twist at t0
	* @param Vf final body twist at tf
	* @param Vdot0 initial body twist derivative at t0
	* @param Vdotf final body twist derivative at tf
	*/
	rxHTransformInterpolator(int type,
							double t0,
							double tf,
							const HTransform& T0,
							const HTransform& Tf,
							const Twist& V0,
							const Twist& Vf,
							const Twist& Vdot0,
							const Twist& Vdotf);
	~rxHTransformInterpolator();
	rxHTransformInterpolator(const rxHTransformInterpolator& rhs);

	/**
	* interpolate the desired transformation, body twist, and derivative at time t
	* @param t time
	* @param T desired transformation
	* @param V desired body twist
	* @param Vdot desired body twist derivative
	*/
	void interpolate(double t, HTransform& T, Twist& V, Twist& Vdot) const;

private:
	rxHTransformInterpolator();
	core::HTransformInterpolator* _impl;
};
} // namespace rxcontrolsdk
} // namespace rlab


#endif
