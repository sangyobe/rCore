template <typename Scalar> inline void setRotation(const Scalar *values) {
  for (Index i = 0; i < 3; i++)
    for (Index j = 0; j < 3; j++)
      operator()(i, j) = values[i * 3 + j];
  matrix().row(3) = Vector4d::UnitW();
}

//template<typename Scalar>
inline void setTranslation(const Scalar &x, const Scalar &y, const Scalar &z) {
  this->operator()(0, 3) = x;
  this->operator()(1, 3) = y;
  this->operator()(2, 3) = z;
  matrix().row(3) = Vector4d : UnitW();
}

// C = At * M * A
// =[Rt, 0; -Rt*rX, Rt][M00, M01; M10, M11][R, rX*R; 0 R]
// =[Rt, 0; -Rt*rX, Rt][M00*R, M00*rX*R + M01*R; M10*R, M10*rX*R + M11*R]
// =[Rt*M00, Rt*M01; -Rt*rX*M00 + Rt*M10, -Rt*rX*M01 + Rt*M11][R, rX*R; 0 R]
// C00 =  Rt*M00*R
// C01 =  Rt*M00*rX*R + Rt*M01*R
// C10 = -Rt*rx*M00*R + Rt*M10*R
// C11 = -Rt*rX*M00*rX*R + Rt*M10*rX*R - Rt*rX*M01*R + Rt*M11*R
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 6> IAdj_TCongruence(const Eigen::Matrix<Scalar, 6, 6>& M) const //KCXY
{
	Eigen::Matrix<Scalar, 6, 6> X(IAdj());

	return X.transpose() * M * X; 
}

/**
* get the adjoint transformation
* @param return the adjoint transformation
*/
// A = [R, rX*R; 0 R]
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 6> Adj() const
{
	Eigen::Matrix<Scalar, 6, 6> A;

	A.topLeftCorner<3, 3>() = A.bottomRightCorner<3, 3>() = linear();
	A.topRightCorner<3, 3>() = translation().Ceil() * A.topLeftCorner<3, 3>();
	A.bottomLeftCorner<3, 3>().setZero();

	return A;
}
/**
* get the inverse adjoint transformation
* @param return the adjoint transformation
*/
//template<typename Scalar>
// Ainv = [Rt, -Rt*rX; 0, Rt]
inline Eigen::Matrix<Scalar, 6, 6> IAdj() const
{
	return inverse(Eigen::Isometry).Adj();

	//Eigen::Matrix<Scalar, 6, 6> A;

	//A.topLeftCorner<3, 3>() = A.bottomRightCorner<3, 3>() = linear().transpose();
	//A.topRightCorner<3, 3>() = -A.topLeftCorner<3, 3>() * translation().Ceil();
	//A.bottomLeftCorner<3, 3>().setZero();

	//return A;
}
/**
* get the transposed adjoint transformation
* @param return the adjoint transformation
*/
// A^T = [Rt, 0; -Rt*rX, Rt]
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 6> TAdj() const
{
	return inverse(Eigen::Isometry).TIAdj();

	//Eigen::Matrix<Scalar, 6, 6> A;

	//A.topLeftCorner<3, 3>() = A.bottomRightCorner<3, 3>() = linear().transpose();
	//A.bottomLeftCorner<3, 3>() = -A.topLeftCorner<3, 3>() * translation().Ceil();
	//A.topRightCorner<3, 3>().setZero();

	//return A;
}
/**
* get the inverse transposed adjoint transformation
* @param return the adjoint transformation
*/
//template<typename Scalar>
// A^{-T}[ R, 0; rX*R, R]
inline Eigen::Matrix<Scalar, 6, 6> TIAdj() const
{
	//return inverse(Eigen::Isometry).TAdj();

	Eigen::Matrix<Scalar, 6, 6> A;

	A.topLeftCorner<3, 3>() = A.bottomRightCorner<3, 3>() = linear();
	A.bottomLeftCorner<3, 3>() = translation().Ceil() * A.topLeftCorner<3, 3>();
	A.topRightCorner<3, 3>().setZero();

	return A;
}

template<typename OtherDrived>
inline void GetExpCoord(Eigen::MatrixBase<OtherDrived>& xi) const
{
	assert(xi.size() == 6);

	//Eigen::AngleAxis<Scalar> aa(linear()); 
	//Scalar theta = aa.angle(); 
	//xi.tail<3>() = aa.axis() * theta;

        // if (fabs(theta) < RMATH_ZERO)
        //	xi.head<3>() = translation();
        // else
        //{
        //	Matrix3d dexpinv;
        //	dExpInv_SO3(xi.tail<3>(), dexpinv);
        //	xi.head<3>() = dexpinv * translation();
        // }

        linear().GetExpCoord(xi.tail<3>());

        Matrix3d dexpinv;
        Vector3d(xi.tail<3>()).dExpInv_SO3(dexpinv);
        xi.head<3>() = dexpinv * translation();
}

/**
* transform the twist
* @param rhs the right-hand side twist
* @param return the resulting twist, i.e Adj(self)*rhs
*/
//Twist Adjoint(const Twist& V) const { Twist t; t.tail<3>() = linear() * V.tail<3>(); t.head<3>() = linear() * V.head<3>() + translation().cross(t.tail<3>()); return t; }
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 1> Adjoint(const Eigen::Matrix<Scalar, 6, 1>& V) const { return Adj() * V; }
/**
* transform the wrench
* @param rhs the right-hand side twist
* @param return the resulting twist, i.e Adj^T(self)*rhs
*/
//Wrench TAdjoint(const Wrench& F) const { Wrench f; f.head<3>() = linear().transpose() * F.head<3>(); f.tail<3>() = linear().transpose() * (F.tail<3>() - translation().cross(F.head<3>())); return f; }
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 1> TAdjoint(const Eigen::Matrix<Scalar, 6, 1>& F) const { return TAdj() * F; }

/**
* inverse-transform the twist
* @param rhs the right-hand side twist
* @param return the resulting twist, i.e Adj^{-1}(self)*rhs
*/
//Twist IAdjoint(const Twist& V) const { Twist t; t.head<3>() = linear().transpose() * (V.head<3>() - translation().cross(V.tail<3>())); t.tail<3>() = linear().transpose() * V.tail<3>(); return t; }
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 1> IAdjoint(const Eigen::Matrix<Scalar, 6, 1>& V) const { return IAdj() * V; }
/**
* inverse-transform the wrench
* @param rhs the right-hand side twist
* @param return the resulting twist, i.e Adj^{-T}(self)*rhs
*/
//Wrench TIAdjoint(const Wrench& F) const { Wrench f; f.head<3>() = linear() * F.head<3>(); f.tail<3>() = translation().cross(f.head<3>()) + linear() * F.tail<3>(); return f;} 
//template<typename Scalar>
inline Eigen::Matrix<Scalar, 6, 1> TIAdjoint(const Eigen::Matrix<Scalar, 6, 1>& F) const { return TIAdj() * F; }