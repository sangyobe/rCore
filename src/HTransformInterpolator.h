/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
// HTransformInterpolator.h: interface for the HTransformInterpolator class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __HTRANSFORMINTERPOLATOR_H__
#define __HTRANSFORMINTERPOLATOR_H__

#include "rcore/VInterpolator.h"

namespace rlab
{
	namespace core
	{

		class RCORE_IMPEXP HTransformInterpolator  
		{
		public:
			~HTransformInterpolator();

			HTransformInterpolator(const Interpolator::INTERPOLATORTYPE type,
				const double t0, 
				const double tf, 
				const HTransform& T0, 
				const HTransform& Tf,
				const Twist& V0, 
				const Twist& Vf, 
				const Twist& Vdot0, 
				const Twist& Vdotf);

			HTransformInterpolator(const HTransformInterpolator& rhs);

			// 	void Reconfigure(const Interpolator::INTERPOLATORTYPE type,
			// 			const double t0, 
			// 			const double tf, 
			// 			const HTransform& T0, 
			// 			const HTransform& Tf,
			// 			const Twist& V0, 
			// 			const Twist& Vf, 
			// 			const Twist& Vdot0, 
			// 			const Twist& Vdotf);

			//HTransformInterpolator(const HTransformInterpolator& Tint);
			//HTransformInterpolator& operator=(const HTransformInterpolator& Tint);
			void interpolate(const double t, HTransform& T, Twist& V, Twist& Vdot) const;

			//double FinalTime() const { return _tf; }

		private:
			HTransformInterpolator() { assert(0); } // : _T0(Isometry3D::Identity()) {}

		private:
			//InterpolationOrder type;
			double			_t0;
			double			_tf;
			HTransform		_T0; //KCX checked

			VInterpolator*	_interpolator;
			mutable dVector			_xi, _xidot, _xiddot;
		};
	}	// core
}	// rlab




#endif // __HTRANSFORMINTERPOLATOR_H__
