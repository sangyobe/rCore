/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rxcontrolsdk/precomp_hdr.h>
#include <rcore/VInterpolator.h>
#include "rxcontrolsdk/rxVInterpolator.h"

namespace rlab {
namespace rxcontrolsdk {

rxVInterpolator::rxVInterpolator(int type, 
								 double t0,
								 double tf,
								 long dim,
								 const dVector& p0,
								 const dVector& pf,
								 const dVector& v0,
								 const dVector& vf,
								 const dVector& a0, 
								 const dVector& af)
{
	_impl = new core::VInterpolator((rlab::core::Interpolator::INTERPOLATORTYPE)type, t0, tf, dim, p0, pf, v0, vf, a0, af);
}

rxVInterpolator::rxVInterpolator(const rxVInterpolator& rhs)
{
	_impl = new core::VInterpolator(*(rhs._impl));
}

rxVInterpolator& rxVInterpolator::operator =(const rxVInterpolator& rhs)
{
	if (this == &rhs)
		return *this;

	
	if (_impl)
		delete _impl;

	_impl = new core::VInterpolator(*(rhs._impl));

	return *this;
}

//void rxVInterpolator::interpolate(double t, dVector& p, dVector& v, dVector& a) const
//{
//	_impl->interpolate(t, p, v, a);
//}
//
//void rxVInterpolator::interpolate(double t, Vector3D& p, Vector3D& v, Vector3D& a) const
//{
//	_impl->interpolate(t, p, v, a);
//}
//
//long rxVInterpolator::dim() const 
//{
//	return _impl->dim();
//}

} // namespace rxcontrolsdk
} // namespace rlab
