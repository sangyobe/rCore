/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
*/
// rxTrapezoidInterpolator.h
//
//////////////////////////////////////////////////////////////////////

#ifndef __RXTRAPEZOIDINTERPOLATOR_H__
#define __RXTRAPEZOIDINTERPOLATOR_H__

#include <rxcontrolsdk/rxInterpolator.h>

namespace rlab {
namespace rxcontrolsdk {
		
class RXCONTROLSDK_IMPEXP rxTrapezoidInterpolator  
{
public:
	virtual ~rxTrapezoidInterpolator();
			
	/**
		* construct the rxTrapezoidInterpolator using boundary conditions
		* @param t0 start time
		* @param tf final time
		* @param vmax maximum velocity
		* @param a maximum acceleration
		* @param p0 initial boundary condition(position)
		* @param pf final boundary condition(position)
		*/
	rxTrapezoidInterpolator( 
				const double t0, 
				const double tf, 
				const double vmax,
				const double a,
				const double p0, 
				const double pf);

	/**
		* interpolate the desired position, velocity, and acceleration at time t
		* @param t time
		* @param p desired position
		* @param v desired velocity
		* @param a desired acceleration
		*/
	void interpolate(const double t, double &p, double &v, double &a) const;

	/*
		* get final time
		* @return final time
		*/
	const double tf() const { return _tf; }

private:
	void _determineCoeff();

private:
	double _t0;
	double _tf;
	double _tc1;
	double _tc2;
	double _a;
	double _vmax;
	double _q0;
	double _qf;
	rxInterpolator* _linearInterpolator[2];
};

} // namespace rxcontrolsdk
} // namespace rlab


#endif // __RXTRAPEZOIDINTERPOLATOR_H__

