/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
*/
// rxVTrapezoidInterpolator.h
//
//////////////////////////////////////////////////////////////////////

#ifndef __RXVTRAPEZOIDINTERPOLATOR_H__
#define __RXVTRAPEZOIDINTERPOLATOR_H__

#include <rlab/math/rMath.h>

#include "rxcontrolsdk/rxTrapezoidInterpolator.h"

namespace rlab {
namespace rxcontrolsdk {

class RXCONTROLSDK_IMPEXP rxVTrapezoidInterpolator  
{
public:
	virtual ~rxVTrapezoidInterpolator();
			
	/**
		* construct the rxVTrapezoidInterpolator using boundary conditions
		* @param t0 start time
		* @param tf final time
		* @param dim variable dimension
		* @param vmax maximum velocity
		* @param a maximum acceleration
		* @param q0 initial position at t0
		* @param qf final position at tf
		*/
	rxVTrapezoidInterpolator( 
				const double t0, 
				const double tf, 
				const long dim,
				const dVector& vmax,
				const dVector& a,
				const dVector& q0,
				const dVector& qf);

	/**
		* interpolate the desired position, velocity, and acceleration at time t
		* @param t time
		* @param p desired position
		* @param v desired velocity
		* @param a desired acceleration
		*/
	void interpolate(const double t, dVector &p, dVector &v, dVector &a) const;	

	/*
		* get final time
		* @return final time
		*/
	const double tf() const { return _tf; }
			
	/*
		* get final time for each trajectory
		* @return final time
		*/
	const dVector& tfv() const { return _tfv; }

private:
	void _determineCoeff();

private:
	double _t0;
	double _tf;
	dVector _tfv;
	long _dim;
	dVector _a;
	dVector _vmax;
	dVector _q0;
	dVector _qf;
	rxTrapezoidInterpolator** _interpolator;
};

} // namespace rxcontrolsdk
} // namespace rlab


#endif // __RXVTRAPEZOIDINTERPOLATOR_H__

