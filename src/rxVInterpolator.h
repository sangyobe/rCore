/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
*/

/** 
@file rxVInterpolator.h
@author Jonghoon Park
@date 2008/9/1
*/
#ifndef __RXVINTERPOLATOR_H__
#define __RXVINTERPOLATOR_H__

#include <rxcontrolsdk/link_pragmas.h>
#include <rlab/math/rMath.h>
#include <rcore/VInterpolator.h>

namespace rlab {
namespace rxcontrolsdk {	

/**
* rxVInterpolator class. 
* @brief class that interpolates a vector variable using polynomial
*/
class RXCONTROLSDK_IMPEXP rxVInterpolator
{
public:
	/**
	* construct the rxVInterpolator using boundary conditions
	* @param type interpolator type of eInterpolatorType
	* @param t0 start time
	* @param tf final time
	* @param dim variable dimension
	* @param p0 initial position at t0
	* @param pf final position at tf
	* @param v0 initial velocity at t0
	* @param vf final velocity at tf
	* @param a0 initial acceleration at t0
	* @param af final acceleration at tf
	*/
	rxVInterpolator(int type, 
					double t0,
					double tf,
					long dim,
					const dVector& p0,
					const dVector& pf,
					const dVector& v0,
					const dVector& vf, 
					const dVector& a0 = dVector(),
					const dVector& af = dVector());

	rxVInterpolator(const rxVInterpolator& rhs);
	rxVInterpolator& operator=(const rxVInterpolator& rhs);

	/**
	* interpolate the desired position, velocity, and acceleration at time t
	* @param t time
	* @param p desired position
	* @param v desired velocity
	* @param a desired acceleration
	*/
	//void interpolate(double t, dVector& p, dVector& v, dVector& a) const;
	//void interpolate(double t, Vector3D& p, Vector3D& v, Vector3D& a) const;
	template<typename OtherDerived>
	void interpolate(double t, Eigen::MatrixBase<OtherDerived>& p, Eigen::MatrixBase<OtherDerived>& v, Eigen::MatrixBase<OtherDerived>& a) const
	{
		_impl->interpolate(t, p, v, a);
	}

	/**
	* get the dimension of interpolated variable
	* @return the dimension of interpolated variable
	*/
	long dim() const { return _impl->dim(); }

private:
	rxVInterpolator();
	core::VInterpolator* _impl;
};
} // namespace rxcontrolsdk
} // namespace rlab


#endif
