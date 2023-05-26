/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
// OrientationInterpolator.h: interface for the OrientationInterpolator class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __ORIENTATIONINTERPOLATOR_H__
#define __ORIENTATIONINTERPOLATOR_H__

#include "rcore/VInterpolator.h"

namespace rlab
{
	namespace core
	{

		class RCORE_IMPEXP OrientationInterpolator  
		{
		public:
			~OrientationInterpolator();

			OrientationInterpolator(const Interpolator::INTERPOLATORTYPE type,
				const double t0, 
				const double tf, 
				const Rotation& R0, 
				const Rotation& Rf,
				const Vector3D& omega0, 
				const Vector3D& omegaf, 
				const Vector3D& alpha0, 
				const Vector3D& alphaf);

			OrientationInterpolator(const OrientationInterpolator& rhs);

			//OrientationInterpolator(const OrientationInterpolator& Rint);
			//OrientationInterpolator& operator=(const OrientationInterpolator& Rint);
			void interpolate(const double t, Rotation& R, Vector3D& omega, Vector3D& alpha)  const;

			//double FinalTime() const { return _tf; }

		private:
			OrientationInterpolator();

		private:
			//InterpolationOrder type;
			double			_t0;
			double			_tf;
			Rotation		_R0;  //KCX checked

			VInterpolator*	_interpolator;
			mutable	Vector3D		_th, _thdot, _thddot;  //KCX checked


		};
	}	// core
}	// rlab


#endif // __ORIENTATIONINTERPOLATOR_H__
