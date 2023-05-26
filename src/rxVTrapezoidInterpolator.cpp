/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rxcontrolsdk/precomp_hdr.h>
#include "rxcontrolsdk/rxVTrapezoidInterpolator.h"

namespace rlab {
namespace rxcontrolsdk {

rxVTrapezoidInterpolator::rxVTrapezoidInterpolator(
				const double t0, 
				const double tf, 
				const long dim,
				const dVector& vmax,
				const dVector& a,
				const dVector& q0, 
				const dVector& qf)
: _t0(t0)
, _tf(tf)
, _dim(dim)
{
	_tfv.resize(_dim);
	_vmax = vmax;
	_a = a;
	_q0 = q0;
	_qf = qf;
	_interpolator = new rxTrapezoidInterpolator*[_dim];
	memset(_interpolator, 0, sizeof(_interpolator[0])*_dim);
	_determineCoeff();
}

rxVTrapezoidInterpolator::~rxVTrapezoidInterpolator()
{
	for (int i=0; i<_dim; i++)
	{
		if (_interpolator[i]) delete _interpolator[i];
	}
	delete [] _interpolator;
}

void rxVTrapezoidInterpolator::interpolate(const double t, dVector &p, dVector &v, dVector &a)  const
{
	for (int i=0; i<_dim; i++)
	{
		_interpolator[i]->interpolate(t, p[i], v[i], a[i]);
	}
}

void rxVTrapezoidInterpolator::_determineCoeff()
{
	double tf_max = 0;

	for (int i=0; i<_dim; i++)
	{
		_interpolator[i] = new rxTrapezoidInterpolator(_t0, _tf, _vmax[i], _a[i], _q0[i], _qf[i]);
		_tfv[i] = _interpolator[i]->tf();
		if (_tfv[i] > tf_max)
			tf_max = _tfv[i];
	}

	_tf = tf_max;
}

} // namespace rxcontrolsdk
} // namespace rlab
