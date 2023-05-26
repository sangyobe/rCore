/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rxcontrolsdk/precomp_hdr.h>	// precompile headers.
#include <rcore/OrientationInterpolator.h>
#include "rxcontrolsdk/rxOrientationInterpolator.h"

namespace rlab {
namespace rxcontrolsdk {

rxOrientationInterpolator::rxOrientationInterpolator(int type, 
													 double t0,
													 double tf,
													 const Rotation& R0,
													 const Rotation& Rf,
													 const Vector3D& omega0,
													 const Vector3D& omegaf,
													 const Vector3D& alpha0,
													 const Vector3D& alphaf)
{
	_impl = new core::OrientationInterpolator((core::Interpolator::INTERPOLATORTYPE)type, t0, tf, R0, Rf, omega0, omega0, alpha0, alphaf);	
}

rxOrientationInterpolator::rxOrientationInterpolator(const rxOrientationInterpolator& rhs)
{
	_impl = new core::OrientationInterpolator(*(rhs._impl));
}

rxOrientationInterpolator::rxOrientationInterpolator()
{

}

rxOrientationInterpolator::~rxOrientationInterpolator()
{
	delete _impl;
}

void rxOrientationInterpolator::interpolate(double t, Rotation& R, Vector3D& omega, Vector3D& alpha) const
{
	_impl->interpolate(t, R, omega, alpha);
}

} // namespace rxcontrolsdk
} // namespace rlab
