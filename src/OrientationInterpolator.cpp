/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rcore/precomp_hdr.h>	// precompile headers.
#include "rcore/OrientationInterpolator.h"

namespace rlab
{
	namespace core
	{

		OrientationInterpolator::OrientationInterpolator(const Interpolator::INTERPOLATORTYPE type,
			const double t0, 
			const double tf, 
			const Rotation& R0, 
			const Rotation& Rf,
			const Vector3D& omega0, 
			const Vector3D& omegaf, 
			const Vector3D& alpha0, 
			const Vector3D& alphaf)
			: _t0(t0)
			, _tf(tf)
			, _interpolator(NULL)
			, _R0(R0)
			, _th(Vector3D::Zero())
			, _thdot(Vector3D::Zero())
			, _thddot(Vector3D::Zero())
		{	
			Vector3D th_0(0, 0, 0);

			Rotation R = R0.transpose() * Rf;
			Vector3D th_f;
			R.GetExpCoord(th_f); 

			Vector3D thdot_0;
			if (type > Interpolator::INTERPOLATORTYPE_LINEAR)
				thdot_0 = omega0;

			Vector3D thdot_f;
			Matrix3D dexpinv_minus;
			if (type > Interpolator::INTERPOLATORTYPE_LINEAR)
			{
				//dExpInv_SO3(-th_f, exp_minus, dexpinv_minus);
				Vector3D(-th_f).dExpInv_SO3(dexpinv_minus);
				//Compute_dExpInv_SO3(-th_f[0], -th_f[1], -th_f[2], exp_minus, dexpinv_minus);
				thdot_f = dexpinv_minus*omegaf; 
			}

			Vector3D thddot_0, thddot_f;
			if (type > Interpolator::INTERPOLATORTYPE_CUBIC)
			{
				Matrix3D ddexp_minus;

				thddot_0 = alpha0;

				//ddt_dExp_SO3(-th_f, -thdot_f, exp_minus, dexp_minus, dexpinv_minus, ddexp_minus);
				Vector3D(-th_f).ddt_dExp_SO3(-thdot_f, ddexp_minus);
				//Compute_ddExp_SO3(-th_f[0], -th_f[1], -th_f[2], -thdot_f[0], -thdot_f[1], -thdot_f[2], 
				//	exp_minus, dexp_minus, dexpinv_minus, ddexp_minus);

				thddot_f = dexpinv_minus*(alphaf - ddexp_minus*thdot_f);
			}

			_interpolator = new VInterpolator(type, _t0, _tf, 3, th_0, th_f, thdot_0, thdot_f, thddot_0, thddot_f);			
		}

		OrientationInterpolator::~OrientationInterpolator()
		{
			if (_interpolator)
				delete _interpolator;
		}

		OrientationInterpolator::OrientationInterpolator(const OrientationInterpolator& rhs)
		{
			_interpolator = new VInterpolator(*rhs._interpolator);
			_t0 = rhs._t0;
			_tf = rhs._tf;
			_R0 = rhs._R0;
			_th = rhs._th;
			_thdot = rhs._thdot;
			_thddot = rhs._thddot;
		}

		void OrientationInterpolator::interpolate(const double t, Rotation& R, Vector3D& omega, Vector3D& alpha) const
		{
			Rotation exp_minus; 
			Matrix3D dexp_minus; 
			Matrix3D dexpinv_minus;
			Matrix3D ddexp_minus;

			_interpolator->interpolate(/*RMATH_SMALLER(t, _tf)*/t, _th, _thdot, _thddot);
			// R = _R0*_th.Exp(); 

			Vector3D(-_th).ddt_dExp_SO3(-_thdot, exp_minus, dexp_minus, dexpinv_minus, ddexp_minus);	
			//Compute_ddExp_SO3(-_th[0], -_th[1], -_th[2], -_thdot[0], -_thdot[1], -_thdot[2], 
			//	exp_minus, dexp_minus, dexpinv_minus, ddexp_minus);	

			//R = _R0*exp_minus.inverse(Eigen::Isometry); 
			R = _R0*exp_minus.transpose(); 
			omega = dexp_minus * _thdot;
			alpha = dexp_minus*_thddot + ddexp_minus * _thdot; 
		}

	}	// core
}	// rlab


