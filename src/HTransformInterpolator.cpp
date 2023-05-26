/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */
#include <rcore/precomp_hdr.h>	// precompile headers.
#include "rcore/HTransformInterpolator.h"

namespace rlab
{
	namespace core
	{

		HTransformInterpolator::HTransformInterpolator(const Interpolator::INTERPOLATORTYPE type, //KCX //YYY //???
			const double t0, 
			const double tf, 
			const HTransform& T0, 
			const HTransform& Tf,
			const Twist& V0, 
			const Twist& Vf, 
			const Twist& Vdot0, 
			const Twist& Vdotf)
			: _t0(t0)
			, _tf(tf)
			, _interpolator(NULL)
			, _T0(T0)
			, _xi(6)
			, _xidot(6)
			, _xiddot(6)
		{	
			Vector6D xi_0;	
			xi_0.setZero();
			//dVector xidot_0 = V0.vector();

			HTransform T = T0.inverse(Eigen::Isometry) * Tf;
			Vector6D xi_f;
			T.GetExpCoord(xi_f);

			Vector6D xidot_0; 
			if (type > Interpolator::INTERPOLATORTYPE_LINEAR)
			{
				xidot_0 = V0;
			}
			else
				xidot_0.setZero();

			Vector6D xidot_f;
			Matrix6D dexpinv_minus;

			if (type > Interpolator::INTERPOLATORTYPE_QUADRATIC)
			{
				Vector6D xi_f_minus(-xi_f);
				xi_f_minus.dExpInv_SE3(dexpinv_minus);
				//Compute_dExpInv_SE3(-xi_f[0], -xi_f[1], -xi_f[2], -xi_f[3], -xi_f[4], -xi_f[5], 
				//					exp_minus, dexpinv_minus);
				xidot_f = dexpinv_minus*Vf; 
			}
			else
				xidot_f.setZero();

			Vector6D xiddot_0, xiddot_f;
			if (type > Interpolator::INTERPOLATORTYPE_CUBIC)
			{
				//HTransform exp_minus; 
				//dMatrix dexp_minus;

				Matrix6D ddexp_minus;

				Twist(-xi_0).ddt_dExpInv_SE3(-xidot_0, ddexp_minus);
				//dVector xiddot_0 = Vdot0.vector() - ddexp_minus*xidot_0;
				xiddot_0 = Vdot0 - ddexp_minus*xidot_0;

				Twist(-xi_f).ddt_dExp_SE3(-xidot_f, ddexp_minus);
				//Compute_ddExp_SE3(-xi_f[0], -xi_f[1], -xi_f[2], -xi_f[3], -xi_f[4], -xi_f[5], 
				//				  -xidot_f[0], -xidot_f[1], -xidot_f[2], -xidot_f[3], -xidot_f[4], -xidot_f[5], 
				//					exp_minus, dexp_minus, dexpinv_minus, ddexp_minus);

				xiddot_f = dexpinv_minus*(Vdotf - ddexp_minus*xidot_f);
			}
			else
			{
				xiddot_0.setZero();
				xiddot_f.setZero();
			}

			_interpolator = new VInterpolator(type, _t0, _tf, 6, xi_0, xi_f, xidot_0, xidot_f, xiddot_0, xiddot_f);			
		}

		HTransformInterpolator::~HTransformInterpolator()
		{
			if (_interpolator)
				delete _interpolator;
		}

		HTransformInterpolator::HTransformInterpolator(const HTransformInterpolator& rhs)
		{
			_interpolator = new VInterpolator(*rhs._interpolator);
			_t0 = rhs._t0;
			_tf = rhs._tf;
			_T0 = rhs._T0;
			_xi = rhs._xi;
			_xidot = rhs._xidot;
			_xiddot = rhs._xiddot;
		}

		void HTransformInterpolator::interpolate(const double t, HTransform& T, Twist& V, Twist& Vdot) const
		{
			_interpolator->interpolate(t/*RMATH_SMALLER(t, _tf)*/, _xi, _xidot, _xiddot);

			HTransform exp_minus; //KCX checked
			Matrix6D dexp_minus; 
			Matrix6D dexpinv_minus;
			Matrix6D ddexp_minus;

			Twist xi_minus(-_xi);
			xi_minus.Exp_SE3(exp_minus);
			xi_minus.dExp_SE3(dexp_minus);
			xi_minus.ddt_dExp_SE3(-_xidot, ddexp_minus);

			//Compute_ddExp_SE3(-_xi[0], -_xi[1], -_xi[2], -_xi[3], -_xi[4], -_xi[5], 
			//				  -_xidot[0], -_xidot[1], -_xidot[2], -_xidot[3], -_xidot[4], -_xidot[5], 							  
			//				  exp_minus, dexp_minus, dexpinv_minus, ddexp_minus);	

			T = _T0*exp_minus.inverse(Eigen::Isometry); 
			V = dexp_minus * _xidot;
			Vdot = dexp_minus * _xiddot + ddexp_minus*_xidot;
		}
	}	// core
}	// rlab

