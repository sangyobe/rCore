/* RoboticsLab, Copyright Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */

#include <rcore/precomp_hdr.h>	// precompile headers.
#include "rcore/VInterpolator.h"

namespace rlab
{
	namespace core
	{

		VInterpolator::VInterpolator(
			const Interpolator::INTERPOLATORTYPE type,
			const double t0, 
			const double tf,
			const long dim,
			const dVector &p0, 
			const dVector &pf,
			const dVector &v0, 
			const dVector &vf, 
			const dVector &a0, 
			const dVector &af)
			: _dim(dim)
			, _t0(t0)
			, _tf(tf)
			, _interpolator(NULL)
		{
			assert(_dim > 0);

			assert(p0.size() == _dim && pf.size() == _dim);
			if (type > Interpolator::INTERPOLATORTYPE_LINEAR)
				assert(v0.size() == _dim);

			if (type > Interpolator::INTERPOLATORTYPE_QUADRATIC)
				assert(vf.size() == _dim);

			if (type > Interpolator::INTERPOLATORTYPE_CUBIC)
				assert(a0.size() == _dim && af.size() == _dim);

			_interpolator = new Interpolator[_dim];

			dVector initial(3);
			dVector final(3);

			initial.setZero();
			final.setZero();

			for (long i = 0; i < _dim; i++)
			{
				initial[0] = p0[i];
				final[0] = pf[i];

				if (type > Interpolator::INTERPOLATORTYPE_LINEAR)
					initial[1] = v0[i];

				if (type > Interpolator::INTERPOLATORTYPE_QUADRATIC)
					final[1] = vf[i];

				//if (type == Interpolator::INTERPOLATORTYPE_QUINTIC ||  type == Interpolator::INTERPOLATORTYPE_JERK)
				if (type > Interpolator::INTERPOLATORTYPE_CUBIC)
				{
					initial[2] = a0[i];
					final[2] = af[i];
				}

				_interpolator[i].reconfigure(type, _t0, _tf, initial, final);
			}
		}

		VInterpolator::~VInterpolator()
		{
			if (_interpolator)
				delete[] _interpolator; 
		}

		VInterpolator::VInterpolator(const VInterpolator &rhs)
			: _dim(rhs._dim)
			, _t0(rhs._t0)
			, _tf(rhs._tf)
		{
			_interpolator = new Interpolator[_dim];
			for (long i = 0; i < _dim; i++)
			{
				_interpolator[i] = rhs._interpolator[i]; 
			}
		}

		VInterpolator &VInterpolator::operator=(const VInterpolator &rhs)
		{
			if (this == &rhs)
				return *this;

			_t0 = rhs._t0; 
			_tf = rhs._tf; 

			if (_dim != rhs._dim)
			{
				_dim = rhs._dim;
				delete[] _interpolator;
				_interpolator = new Interpolator[_dim];
			}

			for (long i = 0; i < _dim; i++)
			{
				_interpolator[i] = rhs._interpolator[i]; 
			}

			return *this;
		}


		//void VInterpolator::interpolate(const double t, dVector &p, dVector &v, dVector &a) const
		//{
		//	p.resize(_dim);
		//	v.resize(_dim);
		//	a.resize(_dim);

		//	for (long i = 0; i < _dim; i++)
		//	{
		//		_interpolator[i].interpolate(t, p[i], v[i], a[i]);
		//	}
		//}

		//void VInterpolator::interpolate(const double t, Vector3D &p, Vector3D &v, Vector3D &a) const
		//{
		//	assert(_dim == 3);

		//	for (long i = 0; i < _dim; i++)
		//	{
		//		_interpolator[i].interpolate(t, p[i], v[i], a[i]);
		//	}
		//}
	}	// core
}	// rlab



