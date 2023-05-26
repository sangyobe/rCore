/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rxcontrolsdk/precomp_hdr.h>	// precompile headers.
#include <rcore/HTransformInterpolator.h>
#include "rxcontrolsdk/rxHTransformInterpolator.h"


namespace rlab {
namespace rxcontrolsdk {

rxHTransformInterpolator::rxHTransformInterpolator()
{
}

rxHTransformInterpolator::rxHTransformInterpolator(int type,
												double t0,
												double tf,
												const HTransform& T0,
												const HTransform& Tf,
												const Twist& V0,
												const Twist& Vf,
												const Twist& Vdot0,
												const Twist& Vdotf)
{
	_impl = new core::HTransformInterpolator((core::Interpolator::INTERPOLATORTYPE)type, t0, tf, T0, Tf, V0, Vf, Vdot0, Vdotf);
}

rxHTransformInterpolator::rxHTransformInterpolator(const rxHTransformInterpolator& rhs)
{
	_impl = new core::HTransformInterpolator((*rhs._impl));
}

rxHTransformInterpolator::~rxHTransformInterpolator()
{
	delete _impl;
}

void rxHTransformInterpolator::interpolate(double t, HTransform &T, Twist &V, Twist &Vdot) const
{
	_impl->interpolate(t, T, V, Vdot);
}

} // namespace rxcontrolsdk
} // namespace rlab
