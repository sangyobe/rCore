/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rxcontrolsdk/precomp_hdr.h>	// precompile headers.
#include <rcore/Interpolator.h>
#include "rxcontrolsdk/rxInterpolator.h"

namespace rlab {
namespace rxcontrolsdk {

rxInterpolator::rxInterpolator(int type, double t0, double tf, const dVector& initial, const dVector& final)
{
	_impl = new core::Interpolator((core::Interpolator::INTERPOLATORTYPE) type, t0, tf, initial, final);
}

rxInterpolator::rxInterpolator(const rxInterpolator& rhs)
{
	_impl = new core::Interpolator(*(rhs._impl));
}

rxInterpolator::~rxInterpolator()
{
	delete _impl;
}

void rxInterpolator::reconfigure(int type, double t0, double tf, const dVector& initial, const dVector& final)
{
	_impl->reconfigure((core::Interpolator::INTERPOLATORTYPE)type, t0, tf, initial, final);
}

void rxInterpolator::interpolate(double t, double &p, double &v, double &a) const
{
	_impl->interpolate(t, p, v, a);
}

} // namespace rxcontrolsdk
} // namespace rlab
