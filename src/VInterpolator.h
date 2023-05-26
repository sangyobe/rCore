/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
// VInterpolator.h: interface for the VInterpolation class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __VINTERPOLATOR_H__
#define __VINTERPOLATOR_H__

#include <rcore/link_pragmas.h>
#include "rcore/Interpolator.h"

namespace rlab
{
	namespace core
	{

		class RCORE_IMPEXP VInterpolator  
		{
		public:
			virtual ~VInterpolator(); 

			VInterpolator(const Interpolator::INTERPOLATORTYPE type,
				const double t0, 
				const double tf, 
				const long dim,
				const dVector& p0, 
				const dVector& pf,
				const dVector& v0, 
				const dVector& vf, 
				const dVector& a0 = dVector(), 
				const dVector& af = dVector());

			VInterpolator(const VInterpolator& rhs);
			VInterpolator& operator=(const VInterpolator& rhs);

			// 	void reconfigure(const Interpolator::INTERPOLATORTYPE type,
			// 			const double t0, 
			// 			const double tf, 
			// 			const long dim,
			// 			const dVector& p0, 
			// 			const dVector& pf,
			// 			const dVector& v0, 
			// 			const dVector& vf, 
			// 			const dVector& a0 = dVector(), 
			// 			const dVector& af = dVector());

			//void interpolate(const double t, dVector& p, dVector& v, dVector& a) const;
			//void interpolate(const double t, Vector3D &p, Vector3D &v, Vector3D &a) const;

			//KCX make sure to resize before call interpolate
			template<typename OtherDerived>
			void interpolate(const double t, Eigen::MatrixBase<OtherDerived>& p, Eigen::MatrixBase<OtherDerived>& v, Eigen::MatrixBase<OtherDerived>& a) const
			{
				assert(p.size() == _dim && v.size() == _dim && a.size() == _dim);  //KCX make sure to resize before call interpolate
				//p.resize(_dim);  //KCX YYY
				//v.resize(_dim);  //KCX YYY
				//a.resize(_dim);  //KCX YYY

				for (long i = 0; i < _dim; i++)
				{
					_interpolator[i].interpolate(t, p[i], v[i], a[i]);
				}
			}

			long dim() const { return _dim; } 
			//double _tf() const { return _tf; }

		private:
			VInterpolator();

		private:
			long			_dim; 
			double			_t0;
			double			_tf;
			Interpolator*	_interpolator;
		};

	}	// core
}	// rlab

#endif // __VINTERPOLATOR_H__
