
inline Scalar at(int i, int j) const { return this->operator()(i,j); }
inline Scalar& at(int i, int j) { return this->operator()(i,j); }
inline Scalar at(int i) const { return this->operator[](i); }
inline Scalar& at(int i) { return this->operator[](i); }

const CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>
operator+(const Scalar& scalar) const
{ return CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>(derived(), internal::scalar_add_op<Scalar>(scalar)); }

friend const CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>
operator+(const Scalar& scalar, const MatrixBase<Derived>& mat)
{ return CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>(mat.derived(), internal::scalar_add_op<Scalar>(scalar)); }

//Vector3D
inline void setXYZ(const Scalar& x_, const Scalar& y_, const Scalar& z_) { assert(size() == 3); x() = x_; y() = y_; z() = z_; }

/**
* get exponential coordinate
* @param th 3d vector
*/
//inline void GetExpCoord(Eigen::Matrix<Scalar, 3, 1>& th) const { Eigen::AngleAxis<Scalar> aa(*this); th = aa.axis() * aa.angle(); }
template<typename OtherDerived>
inline void GetExpCoord(Eigen::MatrixBase<OtherDerived>& th) const { Eigen::AngleAxis<Scalar> aa(*this); th = aa.axis() * aa.angle(); }

/**
* update by rotation about x-axis, i.e. self = self*Rot(x, theta)
* @param theta the rotation angle
*/
inline void XRotate(const Scalar& theta) { *this = *this * Eigen::AngleAxis<Scalar>(theta, Eigen::Vector3d::UnitX()); }	
/**
* update by rotation about y-axis, i.e. self = self*Rot(y, theta)
* @param theta the rotation angle
*/
inline void YRotate(const Scalar& theta) { *this = *this * Eigen::AngleAxis<Scalar>(theta, Eigen::Vector3d::UnitY()); }
/**
* update by rotation about z-axis, i.e. self = self*Rot(z, theta)
* @param theta the rotation angle
*/
inline void ZRotate(const Scalar& theta) { *this = *this * Eigen::AngleAxis<Scalar>(theta, Eigen::Vector3d::UnitZ()); }	

//dVector
/**
* set the elements of the index set 
* @param perm the index set
* @param rhs the source vector
*/
template<typename OtherDerived>
inline void set(const std::vector<long>& perm, const Eigen::MatrixBase<OtherDerived>& rhs)  
{ assert(rhs.size() >= perm.size()); for (Index k = 0; k < perm.size(); k++) this->operator[](perm[k]) = rhs[k]; }
//dMatrix
/**
* set the elements of the submatrix by the row and column index sets by the matrix
* @param rperm the row index set
* @param cperm the column index set
* @param rhs the source matrix
*/
template<typename OtherDerived>
inline void set(const std::vector<long>& rperm, const std::vector<long>& cperm, const Eigen::MatrixBase<OtherDerived>& rhs)
{ assert(rhs.rows() >= rperm.size() && rhs.cols() >= cperm.size()); for (int j = 0; j < cperm.size(); j++) for (int i = 0; i < rperm.size(); i++) operator()(rperm[i], cperm[j]) = rhs(i, j); }
/**
* set the elements of the column submatrix by the row index set by the matrix
* @param rperm the row index set
* @param c the head column index of the submatrix
* @param n the number of columns of the submatrix
* @param rhs the source matrix
*/
template<typename OtherDerived>
inline void set(const std::vector<long>& rperm, const long& c, const long& n, const Eigen::MatrixBase<OtherDerived>& rhs)
{ assert(cols() >= c + n); assert(rhs.rows() >= rperm.size() && rhs.cols() >= n); for (int j = 0; j < n; j++) for (int i = 0; i < rperm.size(); i++) operator()(rperm[i], c + j) = rhs(i, j); }
/**
* set the elements of the row submatrix by the column index set by a matrix
* @param r the head row index of the submatrix
* @param m the number of rows of the submatrix
* @param cperm the column index set
* @param rhs the source matrix
*/
template<typename OtherDerived>
inline void set(const long& r, const long& m, const std::vector<long>& cperm, const Eigen::MatrixBase<OtherDerived>& rhs)
{ assert(rows() >= r + m); assert(rhs.rows() >= m && rhs.cols() >= cperm.size()); for (int j = 0; j < cperm.size(); j++) for (int i = 0; i < m; i++) operator()(r + i, cperm[j]) = rhs(i, j); } 

//dMatrix
template<typename OtherDerived>
EIGEN_STRONG_INLINE void set(const long& i, const long& j, const long& m, const long& n, const Eigen::MatrixBase<OtherDerived>& rhs) 
{ assert(rhs.rows() == m && rhs.cols() == n); Base::block(i, j, m, n) = rhs; } //KCX //???

//dVector
template<typename OtherDerived>
EIGEN_STRONG_INLINE void set(const Index& i, const Index& n, const Eigen::MatrixBase<OtherDerived>& rhs)  
{ assert(rhs.size() == n); segment(i, n) = rhs.head(n); } //KCXY //???

//dVector
/**
* construct the vector (of size n), initialized by float array
* @param n dimension of the vector (n > 0)
* @param rhs 1-d array of float (of size greater than n)
*/
template<typename Type>
inline void set(const Type* values) { for (Index k = 0; k < size(); k++) this->operator[](k) = values[k]; }

//inline bool isValidRotation(Scalar threshold = 1e-4)
//{
//	assert(rows() == cols() == 3);
//
//	int i;
//	for (i = 0; i < 3; i++)
//	{
//		if (std::fabs(col(i).dot(col(i)) - 1) >= threshold) break; // normal
//		if (std::fabs(col(i).dot(col((i + 1)%3))) >= threshold) break; // orhogonal
//	}
//	if (i == 3 && std::fabs(determinant() - 1) < threshold) return true;
//	ucout << "Rotation::Set() is not valid rotation" << std::endl;
//	return false;
//}
/**
* solve the square linear equation. Note that the self matrix is overwritten.
* @param rhs the data. On output, it has the solution, i.e. rhs = self^{-1}*rhs
* @return the error code
*/
// self: mxm, invertable, general or symmetric positive definite
template<typename OtherDerived>
inline void solve(Eigen::MatrixBase<OtherDerived>& mat, const int type = 0 /* GENERAL */) const { if (type == 0) mat = lu().solve(mat); else /* if (type == POSITIVEDEFINITE) */ mat = llt().solve(mat); } //KCX //???

/**
* get the ceiled matrix by ceil operator
* @return the ceil matrix, i.e. [self]
*/
inline Eigen::Matrix<Scalar, 3, 3> Ceil() const
{
	assert(size() == 3);

	Eigen::Matrix<Scalar, 3, 3> S;

	Scalar v1 = x(); 
	Scalar v2 = y(); 
	Scalar v3 = z();

	S(0, 0) = 0;
	S(1, 0) = v3;
	S(2, 0) = -v2;
	S(0, 1) = -v3;
	S(1, 1) = 0;
	S(2, 1) = v1;
	S(0, 2) = v2;
	S(1, 2) = -v1;
	S(2, 2) = 0;

	return S;
}

/**
* get the squared ceiled matrix by ceil operator
* @param rhs the right-hand side 3d vector
* @return the ceil matrix, i.e. [self, rhs]
*/
template<typename OtherDerived>
inline Eigen::Matrix<Scalar, 3, 3> Ceil(const Eigen::MatrixBase<OtherDerived>& w) const
{
	assert(size() == 3 && w.size() == 3);

	Eigen::Matrix<Scalar, 3, 3> S;

	Scalar v1 = x(); 
	Scalar v2 = y(); 
	Scalar v3 = z();
	Scalar w1 = w.x(); 
	Scalar w2 = w.y(); 
	Scalar w3 = w.z();

	Scalar w1v1 = w1 * v1; 
	Scalar w1v2 = w1 * v2;
	Scalar w1v3 = w1 * v3;
	Scalar w2v1 = w2 * v1;
	Scalar w2v2 = w2 * v2;
	Scalar w2v3 = w2 * v3;
	Scalar w3v1 = w3 * v1;
	Scalar w3v2 = w3 * v2;
	Scalar w3v3 = w3 * v3;

	S(0, 0) = -2.0 * (w2v2 + w3v3);
	S(1, 0) = w1v2 + w2v1; 
	S(2, 0) = w1v3 + w3v1;
	S(0, 1) = w1v2 + w2v1;
	S(1, 1) = -2.0 * (w1v1 + w3v3);
	S(2, 1) = w2v3 + w3v2; 
	S(0, 2) = w1v3 + w3v1;
	S(1, 2) = w2v3 + w3v2;
	S(2, 2) = -2.0 * (w1v1 + w2v2); 

	return S;
}

/**
* get the squared ceiled matrix by ceil operator
* @return the ceil matrix, i.e. [self]^2
*/
inline Eigen::Matrix<Scalar, 3, 3> CeilSqr() const
{
	assert(size() == 3);

	Eigen::Matrix<Scalar, 3, 3> S;

	Scalar v1 = x(); 
	Scalar v2 = y(); 
	Scalar v3 = z();

	Scalar v1sqr = v1 * v1;
	Scalar v2sqr = v2 * v2;
	Scalar v3sqr = v3 * v3;

	Scalar v12 = v1 * v2;
	Scalar v13 = v1 * v3;
	Scalar v23 = v2 * v3;

	S(0, 0) = -v2sqr - v3sqr;
	S(1, 0) = v12;
	S(2, 0) = v13;
	S(0, 1) = v12;
	S(1, 1) = -v1sqr - v3sqr;
	S(2, 1) = v23;
	S(0, 2) = v13;
	S(1, 2) = v23;
	S(2, 2) = -v1sqr - v2sqr;

	return S;
}

void Exp_SO3(Eigen::Block<Eigen::Matrix<Scalar, 4, 4>, 3, 3>& exp)
{
	const Vector3D& w = *this;

	double wnorm = w.norm();
	exp.setIdentity();

	if (wnorm > RMATH_ZERO)
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);
    
		double alpha = s * c;
		double beta = s * s;

		exp += alpha * w.Ceil() + beta / 2.0 * w.CeilSqr();
	}
}

void dExp_SO3(Eigen::Block<Eigen::Matrix<Scalar, 4, 4>, 3, 3>& exp, Eigen::Matrix<Scalar, 3, 3>& dexp)
{
	const Vector3D& w = *this;

	double wnorm = w.norm(); 

	exp.setIdentity(); 
	dexp.setIdentity();	

	if (wnorm > RMATH_ZERO)           
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);

		double alpha = s * c;
		double beta = s * s;
		double half_beta = beta/2;

		Matrix3D W = w.Ceil();
		Matrix3D W2 = w.CeilSqr();

		exp += alpha * W + half_beta * W2;
		dexp += half_beta*W + (1.0 - alpha) / (wnorm*wnorm) * W2;
	}
}

void dExpInv_SO3(Eigen::Matrix<Scalar, 3, 3>& exp, Eigen::Matrix<Scalar, 3, 3>& dexpinv)
{
	const Vector3D& w = *this;

	double wnorm = w.norm();

	exp.setIdentity(); 
	dexpinv.setIdentity();

	if (wnorm > RMATH_ZERO)
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);

		double alpha = s * c;
		double beta = s * s;
		double gamma = c / s;

		Matrix3D W = w.Ceil();
		Matrix3D W2 = w.CeilSqr();

		exp += alpha * W + beta / 2 * W2;
		dexpinv += (1 - gamma) / (wnorm*wnorm) * W2 - W/2;	
	}
}

void ddt_dExp_SO3(const Eigen::Matrix<Scalar, 3, 1>& wdot,
				   Eigen::Matrix<Scalar, 3, 3>& exp, Eigen::Matrix<Scalar, 3, 3>& dexp, Eigen::Matrix<Scalar, 3, 3>& dexpinv, Eigen::Matrix<Scalar, 3, 3>& ddt_dexp)
{
	const Vector3D& w = *this;

	double wnorm = w.norm();
	
	exp.setIdentity(); 
	dexp.setIdentity();
	dexpinv.setIdentity();

	if (wnorm < RMATH_ZERO)
		ddt_dexp = 0.5*wdot.Ceil();

	else
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);

		double alpha = s * c;
		double beta = s * s;
		double half_beta = beta / 2;
		double gamma = c / s;

		double wnorm_sqr = wnorm * wnorm;
		double temp = (1.0 - alpha) / wnorm_sqr;
		double w_wdot = w.dot(wdot);

		Matrix3D W = w.Ceil();
		Matrix3D W2 = w.CeilSqr();

		exp += alpha * W + half_beta * W2;
		dexp += half_beta * W + temp * W2;
		dexpinv += (1.0 - gamma) / wnorm_sqr * W2 - 0.5 * W;	
		ddt_dexp = half_beta*wdot.Ceil() + (alpha - beta) / wnorm_sqr * w_wdot * W + temp * w.Ceil(wdot) + (half_beta - 3 * temp) * w_wdot / wnorm_sqr * W2; 
	}
}

void ddt_dExpInv_SO3(const Eigen::Matrix<Scalar, 3, 1>& wdot,
				   Eigen::Matrix<Scalar, 3, 3>& exp, Eigen::Matrix<Scalar, 3, 3>& dexp, Eigen::Matrix<Scalar, 3, 3>& dexpinv, Eigen::Matrix<Scalar, 3, 3>& ddt_dexpinv)
{
	const Vector3D& w = *this;

	double wnorm = w.norm();
	
	exp.setIdentity(); 
	dexp.setIdentity();
	dexpinv.setIdentity();

	ddt_dexpinv = -0.5 * wdot.Ceil();

	if (wnorm > RMATH_ZERO)
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);

		double alpha = s * c;	
		double beta = s * s;
		double gamma = c / s;
		double half_beta = beta / 2;

		double wnorm_sqr = wnorm * wnorm;
		double temp = (1 - gamma) / wnorm_sqr;
		double w_wdot = w.dot(wdot);

		Matrix3D W = w.Ceil();
		Matrix3D W2 = w.CeilSqr();

		exp += alpha * W + half_beta * W2;
		dexp += half_beta * W + (1 - alpha) / wnorm_sqr * W2;
		dexpinv += temp * W2 - 0.5 * W;
		ddt_dexpinv += temp * w.Ceil(wdot) + (1 / beta + gamma - 2) / wnorm_sqr * w_wdot / wnorm_sqr * W2; 
	}
}

void dExp_SO3(Eigen::Matrix<Scalar, 3, 3>& dexp)
{
	const Vector3D& w = *this;

	double wnorm = w.norm(); 

	dexp.setIdentity();	

	if (wnorm > RMATH_ZERO)           
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);

		double alpha = s * c;
		double beta = s * s;
		double half_beta = beta / 2;

		dexp += half_beta * w.Ceil() + (1.0 - alpha) / (wnorm * wnorm) * w.CeilSqr();
	}
}

void dExpInv_SO3(Eigen::Matrix<Scalar, 3, 3>& dexpinv)
{
	const Vector3D& w = *this;

	double wnorm = w.norm();

	dexpinv.setIdentity();

	if (wnorm > RMATH_ZERO)
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);

		double gamma = c / s;
		double wnorm_sqr = wnorm * wnorm;

		//Eigen::Matrix3d temp1 = -0.5*w.Ceil(); temp1.print("-1/2*w.Ceil()");
		//Eigen::Matrix3d temp2 = (1 - gamma)/wnorm_sqr*w.CeilSqr(); temp2.print("(1 - gamma)/wnorm_sqr*w.CeilSqr()");

		dexpinv += -0.5 * w.Ceil() + (1 - gamma) / wnorm_sqr * w.CeilSqr();	
	}
}

void ddt_dExp_SO3(const Eigen::Matrix<Scalar, 3, 1>& wdot, Eigen::Matrix<Scalar, 3, 3>& ddt_dexp)
{
	const Vector3D& w = *this;

	double wnorm = w.norm();

	if (wnorm > RMATH_ZERO)
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		
		double beta = s * s;
		double half_beta = beta / 2;

		//ddt_dexp = half_beta*wdot.Ceil();
	
		double c = std::cos(half_wnorm);

		double alpha = s * c;
		double gamma = c / s;

		double wnorm_sqr = wnorm * wnorm;
		double temp = (1.0 - alpha) / wnorm_sqr;
		double w_wdot = w.dot(wdot);

		ddt_dexp = half_beta * wdot.Ceil() + (alpha - beta) / wnorm_sqr * w_wdot * w.Ceil() + temp * w.Ceil(wdot) + (half_beta - 3 * temp) * w_wdot / wnorm_sqr * w.CeilSqr(); 
	}
	else
	{	
		ddt_dexp = 0.5*wdot.Ceil();
	}
}

void ddt_dExpInv_SO3(const Eigen::Matrix<Scalar, 3, 1>& wdot, Eigen::Matrix<Scalar, 3, 3>& ddt_dexpinv)
{
	const Vector3D& w = *this;

	double wnorm = w.norm();
	
	ddt_dexpinv = -0.5 * wdot.Ceil();

	if (wnorm > RMATH_ZERO)
	{
		double half_wnorm = wnorm / 2;
		double s = std::sin(half_wnorm) / half_wnorm;
		double c = std::cos(half_wnorm);

		double alpha = s * c;	
		double beta = s * s;
		double gamma = c / s;
		double half_beta = beta / 2;

		double wnorm_sqr = wnorm * wnorm;
		double w_wdot = w.dot(wdot);

		ddt_dexpinv += (1 - gamma) / wnorm_sqr * w.Ceil(wdot) + (1 / beta + gamma - 2) / wnorm_sqr * w_wdot / wnorm_sqr * w.CeilSqr(); 
	}
}

//Twist
/**
* get the adjoint operator
* @return the adjoint operator
*/
// Twist(v, w)
// [ wX, vX; 0 wX] *[ v1; w1] = [wX*v1 + vX*w1; wX*w1]
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 6> Cross() const
{			
	assert(size() == 6);

	Eigen::Matrix<Scalar, 6, 6> A;

	A.topLeftCorner<3, 3>() = A.bottomRightCorner<3, 3>() =  tail<3>().Ceil();
	A.topRightCorner<3, 3>() =  head<3>().Ceil();
	A.bottomLeftCorner<3, 3>().setZero();

	return A;    
}

//template<typename Scalar>
template<typename OtherDerived>
inline Eigen::Matrix<Scalar, 6, 1> Cross(const Eigen::MatrixBase<OtherDerived>& V) const 
{ 
	assert(size() == 6 && V.size() == 6);

	Eigen::Matrix<Scalar, 6, 1> v; 

	v.head<3>() = tail<3>().cross(V.head<3>()) + head<3>().cross(V.tail<3>()); 
	v.tail<3>() = tail<3>().cross(V.tail<3>()); 

	return v; 
}

//template<typename Scalar>
void Exp_SE3(Eigen::Transform<Scalar, 3, Eigen::Isometry>& exp)
{
	assert(size() == 6);

	const Twist& xi = *this;
	Scalar wnorm = xi.tail<3>().norm();
	
	if (wnorm < RMATH_ZERO)
	{
		exp.linear().setIdentity();
		exp.translation() = xi.head<3>();
	}
	else
	{
		Matrix3D dexp;
		Vector3D(xi.tail<3>()).dExp_SO3(exp.linear(), dexp);
		exp.translation() = dexp*xi.head<3>();
	}
}

void dExpInv_SE3(Eigen::Matrix<Scalar, 6, 6>& dexpinv)
{
	assert(size() == 6);

	const Twist& xi = *this;
	Rotation R;
	Matrix3D dexp_so, dexpinv_so, ddt_dexpinv_so;
	
	//dExpInv_SO3(xi.tail<3>(), dexpinv_so);
	//ddt_dExpInv_SO3(xi.tail<3>(), xi.head<3>(), ddt_dexpinv_so);
	Vector3D(xi.tail<3>()).ddt_dExpInv_SO3(xi.head<3>(), R, dexp_so, dexpinv_so, ddt_dexpinv_so);
		
	dexpinv.topLeftCorner<3, 3>() = dexpinv.bottomRightCorner<3, 3>() = dexpinv_so;
	dexpinv.topRightCorner<3, 3>() = ddt_dexpinv_so;
	dexpinv.bottomLeftCorner<3, 3>().setZero();
}

void dExp_SE3(Eigen::Matrix<Scalar, 6, 6>& dexp)
{
	assert(size() == 6);

	const Twist& xi = *this;
	Rotation R;
	Matrix3D dexp_so, dexpinv_so, ddt_dexp_so;

	Vector3D(xi.tail<3>()).ddt_dExp_SO3(xi.head<3>(), R, dexp_so, dexpinv_so, ddt_dexp_so);

	dexp.topLeftCorner<3, 3>() = dexp.bottomRightCorner<3, 3>() = dexp_so;
	dexp.topRightCorner<3, 3>() = ddt_dexp_so;
	dexp.bottomLeftCorner<3, 3>().setZero();
}

void ddt_dExpInv_SE3(const Eigen::Matrix<Scalar, 6, 1>& xidot, Eigen::Matrix<Scalar, 6, 6>& ddt_dexpinv)
{
	assert(size() == 6);

	const Twist& xi = *this;
	Scalar wnorm = xi.tail<3>().norm();
	
	Matrix3D ddt_dexpinv_so = -0.5 * xidot.tail<3>().Ceil();

	Matrix3D D = -0.5 * xidot.head<3>().Ceil();

	if (wnorm > RMATH_ZERO)
	{
		Scalar half_wnorm = 0.5 * wnorm;
		Scalar s = std::sin(half_wnorm) / half_wnorm;
		Scalar c = std::cos(half_wnorm);

		Scalar alpha = s * c;	
		Scalar beta = s * s;
		Scalar gamma = c / s;
		Scalar half_beta = beta / 2;

		Scalar wnorm_sqr = wnorm * wnorm;
		Scalar wnorm_quad = wnorm_sqr * wnorm_sqr;

		Scalar w_v = xi.tail<3>().dot(xi.head<3>());
		Scalar wdot_v = xidot.tail<3>().dot(xi.head<3>());
		Scalar w_vdot = xi.tail<3>().dot(xidot.head<3>());
		Scalar w_wdot = xi.tail<3>().dot(xidot.tail<3>());

		Scalar temp = (1 - gamma) / wnorm_sqr;
		Scalar temp2 = (1/beta + gamma - 2) / wnorm_quad;

		Scalar temp4 = w_v * w_wdot/wnorm_sqr;

		Matrix3D W2 = xi.tail<3>().CeilSqr();
		Matrix3D W3 = xi.tail<3>().Ceil(xidot.tail<3>());
		ddt_dexpinv_so += temp*W3 + temp2*w_wdot * W2; 

		D += temp * (xidot.head<3>().Ceil(xi.tail<3>()) + xi.head<3>().Ceil(xidot.tail<3>()))
			+ temp2 * (w_v * W3 + (wdot_v + w_vdot - 3 * temp4) * W2  + w_wdot * xi.head<3>().Ceil(xi.tail<3>())) 
			+ 2.0 / wnorm_sqr * (1.0 - gamma/beta) / wnorm_sqr * temp4 * W2;
			//- ( (3 + 2*gamma)/beta + 3*gamma - 8)/wnorm_quad*w_wdot/wnorm_sqr*w_v*W2;
			//+ temp2*((wdot_v + w_vdot)*W2 + w_v*xi.tail<3>().Ceil(xidot.tail<3>()) + w_wdot*xi.head<3>().Ceil(xi.tail<3>()))
			//- ( (3 + 2*gamma)/beta + 3*gamma - 8)/wnorm_quad*w_wdot/wnorm_sqr*w_v*W2;

	}
	else
	{
		D += (1.0/12.0) * xi.head<3>().Ceil(xidot.tail<3>());
	}

	ddt_dexpinv.topLeftCorner<3, 3>() = ddt_dexpinv.bottomRightCorner<3, 3>() = ddt_dexpinv_so;
	ddt_dexpinv.topRightCorner<3, 3>() = D;
	ddt_dexpinv.bottomLeftCorner<3, 3>().setZero();
}

void ddt_dExp_SE3(const Eigen::Matrix<Scalar, 6, 1>& xidot, Eigen::Matrix<Scalar, 6, 6>& ddt_dexp)
{
	assert(size() == 6);

	const Twist& xi = *this;
	Scalar wnorm = xi.tail<3>().norm();
	Matrix3D ddt_dexp_so;
	Matrix3D C;	

	if (wnorm > RMATH_ZERO)
	{
		Scalar half_wnorm = 0.5 * wnorm;
		Scalar s = std::sin(half_wnorm) / half_wnorm;
		Scalar c = std::cos(half_wnorm);

		Scalar alpha = s * c;	
		Scalar beta = s * s;
		Scalar half_beta = beta / 2;

		Scalar wnorm_sqr = wnorm * wnorm;
		Scalar wnorm_quad = wnorm_sqr * wnorm_sqr;

		Scalar w_v = xi.tail<3>().dot(xi.head<3>());
		Scalar wdot_v = xidot.tail<3>().dot(xi.head<3>());
		Scalar w_vdot = xi.tail<3>().dot(xidot.head<3>());
		Scalar w_wdot = xi.tail<3>().dot(xidot.tail<3>());

		Scalar temp = (1 - alpha) / wnorm_sqr;
		Scalar temp2 = (alpha - beta) / wnorm_sqr;	// corrected at 20070928 
		Scalar temp3 = (half_beta - 3 * temp) / wnorm_sqr;

		Scalar temp4 = w_v * w_wdot / wnorm_sqr;

		Matrix3D W = xi.tail<3>().Ceil();
		Matrix3D W2 = xi.tail<3>().CeilSqr();
		Matrix3D W3 = xi.tail<3>().Ceil(xidot.tail<3>());
		
		ddt_dexp_so = half_beta*xidot.tail<3>().Ceil() + temp * xi.tail<3>().Ceil(xidot.tail<3>()) + temp2 * w_wdot*W + temp3 * w_wdot * W2;;

		C = half_beta * (xidot.head<3>().Ceil() - temp4*W) 
			+ temp * (xidot.head<3>().Ceil(xi.tail<3>()) + xi.head<3>().Ceil(xidot.tail<3>()) + temp4 * W)
			+ temp2 * (w_v * xidot.tail<3>().Ceil() + (wdot_v + w_vdot - 4 * temp4) * W  + w_wdot * xi.head<3>().Ceil() + temp4*W2)
			+ temp3 * (w_v * W3 + (wdot_v + w_vdot - 5*temp4) * W2 + w_wdot * xi.head<3>().Ceil(xi.tail<3>()));
	}
	else
	{
		ddt_dexp_so = 0.5 * xi.head<3>().Ceil();
		C = 0.5 * xidot.head<3>().Ceil() + (1.0/6.0) * xi.head<3>().Ceil(xidot.tail<3>());
	}

	ddt_dexp.topLeftCorner<3, 3>() = ddt_dexp.bottomRightCorner<3, 3>() = ddt_dexp_so;
	ddt_dexp.topRightCorner<3, 3>() = C;
	ddt_dexp.bottomLeftCorner<3, 3>().setZero();
}