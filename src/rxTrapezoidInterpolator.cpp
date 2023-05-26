/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rxcontrolsdk/precomp_hdr.h>	// precompile headers.
#include "rxcontrolsdk/rxTrapezoidInterpolator.h"

namespace rlab {
namespace rxcontrolsdk {

rxTrapezoidInterpolator::rxTrapezoidInterpolator(
				const double t0, 
				const double tf, 
				const double vmax,
				const double a,
				const double p0, 
				const double pf)
: _t0(t0)
, _tf(tf)
, _tc1(0)
, _tc2(0)
, _q0(0)
, _qf(0)
, _vmax(vmax)
, _a(a)
{
	_linearInterpolator[0] = NULL;
	_linearInterpolator[1] = NULL;
	_q0 = p0;
	_qf = pf;
	_determineCoeff();
}

rxTrapezoidInterpolator::~rxTrapezoidInterpolator()
{
	if (_linearInterpolator[0]) delete _linearInterpolator[0];
	if (_linearInterpolator[1]) delete _linearInterpolator[1];
}

void rxTrapezoidInterpolator::interpolate(const double t, double &p, double &v, double &a)  const
{
	double t_ = t;
	double dir = ((_qf - _q0) >= 0 ? 1.0 : -1.0);
	if (t_ > _tf) t_ = _tf;
	if (t_ < _t0) t_ = _t0;

	if (t_ >= _t0 && t_ < _tc1)
	{
		a = _a * dir;
		v = a * (t_ - _t0);
		p = _q0 + 0.5 * a * (t_ - _t0) * (t_ - _t0);
	}
	else if (t_ >= _tc1 && t_ < _tc2)
	{
		a = 0;
		v = _vmax * dir;
		p = _q0 + 0.5 * _a * dir * (_tc1 -_t0) * (_tc1 -_t0) + v * (t_ - _tc1);
	}
	else
	{
		a = -_a * dir;
		v = -a * (_tf - t_);
		p = _qf + 0.5 * a * (_tf - t_) * (_tf - t_);
	}
}

void rxTrapezoidInterpolator::_determineCoeff()
{
	double vmaxsqr = _vmax * _vmax;
	double dir = ((_qf - _q0) >= 0 ? 1.0 : -1.0);

	if ((vmaxsqr / (2 * _a)) >= (abs(_qf - _q0) / 2))
	{
		_tc1 = _tc2 = _t0 + sqrt(abs(_qf - _q0) / _a);
		_tf = _t0 + 2 * sqrt(abs(_qf - _q0) / _a);
	}
	else
	{
		_tc1 = _t0 + (_vmax / _a);
		_tc2 = _tc1 + (abs(_qf - _q0) - vmaxsqr / _a) / _vmax;
		_tf = _tc2 + (_vmax / _a);
	}

	_vmax = _a * (_tc1 - _t0);

	dVector initial(3);
	initial.setZero();
	initial[0] = _q0;
	dVector final(3);
	final.setZero();
	final[0] = _qf;
	dVector viaposition(3);
	viaposition.setZero();
	viaposition[0] = _q0 + 0.5 * _a * (_tc1 - _t0) * (_tc1 - _t0) * dir;
	_linearInterpolator[0] = new rxInterpolator(eInterpolatorType_Linear, _t0, _tc1, initial, viaposition);
	viaposition[0] = _qf - 0.5 * _a * (_tf - _tc2) * (_tf - _tc2) * dir;
	_linearInterpolator[1] = new rxInterpolator(eInterpolatorType_Linear, _tc2, _tf, viaposition, final);
}

} // namespace rxcontrolsdk
} // namespace rlab
