//dVector
inline void reduce(const Index& i, const Index& n) 
{ assert(size() >= i + n); Index diff = size() - (i + n); segment(i, diff) = tail(diff); conservativeResize(size() - n); }	

//dMatrix
inline void rexpand(const Index& m) { conservativeResize(rows() + m, Eigen::NoChange); bottomRows(m).setZero(); }

//template<typename OtherDerived>
//EIGEN_STRONG_INLINE void add(const Index& i, const Index& n, const Eigen::MatrixBase<OtherDerived>& rhs)  
//{ assert(rhs.size() == n); segment(i, n) += rhs.head(n); } //KCXY //???				

//template<typename OtherDerived>
//EIGEN_STRONG_INLINE void sub(const Index& i, const Index& n, const Eigen::MatrixBase<OtherDerived>& rhs)  
//{ assert(rhs.size() == n); segment(i, n) -= rhs.head(n); } //KCXY //???

//dVector
/**
* get the subvector with the index vector
* @param perm the index set
* @return the subvector
*/
inline Eigen::Matrix<_Scalar, _Rows, 1> permute2(const std::vector<long>& perm) const 
{ Eigen::Matrix<_Scalar, _Rows, 1> vec(perm.size()); for (int k = 0; k < perm.size(); k++) vec[k] = operator[](perm[k]); return vec; }

/**
* get the column submatrix with the column index vector
* @param perm the column index set
* @return the submatrix
*/
inline Eigen::Matrix<_Scalar, _Rows, _Cols> postpermute(const std::vector<long>& perm) const 
{ Eigen::Matrix<_Scalar, _Rows, _Cols> A(rows(), perm.size()); for (int k = 0; k < perm.size(); k++) A.col(k) = col(perm[k]); return A; }
/**
* get the row submatrix with the row index vector
* @param perm the row index set
* @return the submatrix
*/
inline Eigen::Matrix<_Scalar, _Rows, _Cols> prepermute(const std::vector<long>& perm) const 
{ Eigen::Matrix<_Scalar, _Rows, _Cols> A(perm.size(), cols()); for (int k = 0; k < perm.size(); k++) A.row(k) = row(perm[k]); return A; }
/**
* get the square submatrix with the index vector
* @param perm the index set
* @return the submatrix
*/
inline Eigen::Matrix<_Scalar, _Rows, _Cols> permute(const std::vector<long>& perm) const 
{ return postpermute(perm).prepermute(perm); } // select cols (post) then select rows (pre)

/**
* get the inverse matrix
* @param lhs the inverse matrix
* @return the error code
*/
//void inv(Eigen::Matrix<_Scalar, _Rows, _Cols>& lhs, const int type = 0 /* GENERAL */) const 
//{ assert(rows() == cols()); Eigen::Matrix<_Scalar, _Rows, _Cols> m; if (type == GENERAL) m = lu().solve(Eigen::MatrixXd::Identity(rows(), cols())); else /* if (type == POSITIVEDEFINITE) */ m = llt().solve(Eigen::MatrixXd::Identity(rows(), cols())); lhs = m; } //KCX //???
inline void inv(Eigen::Matrix<_Scalar, _Rows, _Cols>& lhs, const int type = 0 /* GENERAL */) const 
{ assert(rows() == cols()); if (type == rlab::math::GENERAL) lhs = lu().solve(Eigen::MatrixXd::Identity(rows(), cols())); else /* if (type == POSITIVEDEFINITE) */ lhs = llt().solve(Eigen::MatrixXd::Identity(rows(), cols())); } //KCX //???

/**
* get the pseudo-inverse matrix
* @param lhs the pseudo-inverse matrix
*/
//inline void pinv(Eigen::Matrix<_Scalar, _Rows, _Cols>& lhs, const double& threshold = 1e-8) const
//{ Eigen::Matrix<_Scalar, _Rows, _Cols> m = jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(rows(), rows())); lhs = m; /* SVD svd(*this, threshold, 1); svd.Invert(minv); */ } //KCX //???
inline void pinv(Eigen::Matrix<_Scalar, _Rows, _Cols>& lhs, const double& threshold = 1e-8) const
{ /*lhs = jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(rows(), rows())); */ rlab::math::SVD svd(*this, threshold, 1); svd.Invert(lhs); } //KCX //???

/**
 * get exponential coordinate
 * @param th 3d vector
 */
// inline void GetExpCoord(Eigen::Matrix<Scalar, 3, 1>& th) const {
// Eigen::AngleAxis<Scalar> aa(*this); th = aa.axis() * aa.angle(); }
template <typename OtherDerived>
inline void GetExpCoord(Eigen::MatrixBase<OtherDerived> &th) const {
  Eigen::AngleAxis<Scalar> aa(*this);
  th = aa.axis() * aa.angle();
}

/**
 * update by rotation about x-axis, i.e. self = self*Rot(x, theta)
 * @param theta the rotation angle
 */
inline void XRotate(const Scalar &theta) {
  *this = *this * Eigen::AngleAxis<Scalar>(theta, Eigen::Vector3d::UnitX());
}
/**
 * update by rotation about y-axis, i.e. self = self*Rot(y, theta)
 * @param theta the rotation angle
 */
inline void YRotate(const Scalar &theta) {
  *this = *this * Eigen::AngleAxis<Scalar>(theta, Eigen::Vector3d::UnitY());
}
/**
 * update by rotation about z-axis, i.e. self = self*Rot(z, theta)
 * @param theta the rotation angle
 */
inline void ZRotate(const Scalar &theta) {
  *this = *this * Eigen::AngleAxis<Scalar>(theta, Eigen::Vector3d::UnitZ());
}

void Exp_SO3(Eigen::Block<Eigen::Matrix<Scalar, 4, 4>, 3, 3> &exp) {
  const Vector3d &w = *this;

  double wnorm = w.norm();
  exp.setIdentity();

  if (wnorm > RMATH_ZERO) {
    double half_wnorm = wnorm / 2;
    double s = std::sin(half_wnorm) / half_wnorm;
    double c = std::cos(half_wnorm);

    double alpha = s * c;
    double beta = s * s;

    exp += alpha * w.Ceil() + beta / 2.0 * w.CeilSqr();
  }
}

void dExp_SO3(Eigen::Block<Eigen::Matrix<Scalar, 4, 4>, 3, 3> &exp,
              Eigen::Matrix<Scalar, 3, 3> &dexp) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  exp.setIdentity();
  dexp.setIdentity();

  if (wnorm > RMATH_ZERO) {
    double half_wnorm = wnorm / 2;
    double s = std::sin(half_wnorm) / half_wnorm;
    double c = std::cos(half_wnorm);

    double alpha = s * c;
    double beta = s * s;
    double half_beta = beta / 2;

    Matrix3d W = w.Ceil();
    Matrix3d W2 = w.CeilSqr();

    exp += alpha * W + half_beta * W2;
    dexp += half_beta * W + (1.0 - alpha) / (wnorm * wnorm) * W2;
  }
}

void dExpInv_SO3(Eigen::Matrix<Scalar, 3, 3> &exp,
                 Eigen::Matrix<Scalar, 3, 3> &dexpinv) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  exp.setIdentity();
  dexpinv.setIdentity();

  if (wnorm > RMATH_ZERO) {
    double half_wnorm = wnorm / 2;
    double s = std::sin(half_wnorm) / half_wnorm;
    double c = std::cos(half_wnorm);

    double alpha = s * c;
    double beta = s * s;
    double gamma = c / s;

    Matrix3d W = w.Ceil();
    Matrix3d W2 = w.CeilSqr();

    exp += alpha * W + beta / 2 * W2;
    dexpinv += (1 - gamma) / (wnorm * wnorm) * W2 - W / 2;
  }
}

void ddt_dExp_SO3(const Eigen::Matrix<Scalar, 3, 1> &wdot,
                  Eigen::Matrix<Scalar, 3, 3> &exp,
                  Eigen::Matrix<Scalar, 3, 3> &dexp,
                  Eigen::Matrix<Scalar, 3, 3> &dexpinv,
                  Eigen::Matrix<Scalar, 3, 3> &ddt_dexp) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  exp.setIdentity();
  dexp.setIdentity();
  dexpinv.setIdentity();

  if (wnorm < RMATH_ZERO)
    ddt_dexp = 0.5 * wdot.Ceil();

  else {
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

    Matrix3d W = w.Ceil();
    Matrix3d W2 = w.CeilSqr();

    exp += alpha * W + half_beta * W2;
    dexp += half_beta * W + temp * W2;
    dexpinv += (1.0 - gamma) / wnorm_sqr * W2 - 0.5 * W;
    ddt_dexp = half_beta * wdot.Ceil() +
               (alpha - beta) / wnorm_sqr * w_wdot * W + temp * w.Ceil(wdot) +
               (half_beta - 3 * temp) * w_wdot / wnorm_sqr * W2;
  }
}

void ddt_dExpInv_SO3(const Eigen::Matrix<Scalar, 3, 1> &wdot,
                     Eigen::Matrix<Scalar, 3, 3> &exp,
                     Eigen::Matrix<Scalar, 3, 3> &dexp,
                     Eigen::Matrix<Scalar, 3, 3> &dexpinv,
                     Eigen::Matrix<Scalar, 3, 3> &ddt_dexpinv) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  exp.setIdentity();
  dexp.setIdentity();
  dexpinv.setIdentity();

  ddt_dexpinv = -0.5 * wdot.Ceil();

  if (wnorm > RMATH_ZERO) {
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

    Matrix3d W = w.Ceil();
    Matrix3d W2 = w.CeilSqr();

    exp += alpha * W + half_beta * W2;
    dexp += half_beta * W + (1 - alpha) / wnorm_sqr * W2;
    dexpinv += temp * W2 - 0.5 * W;
    ddt_dexpinv += temp * w.Ceil(wdot) +
                   (1 / beta + gamma - 2) / wnorm_sqr * w_wdot / wnorm_sqr * W2;
  }
}

void dExp_SO3(Eigen::Matrix<Scalar, 3, 3> &dexp) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  dexp.setIdentity();

  if (wnorm > RMATH_ZERO) {
    double half_wnorm = wnorm / 2;
    double s = std::sin(half_wnorm) / half_wnorm;
    double c = std::cos(half_wnorm);

    double alpha = s * c;
    double beta = s * s;
    double half_beta = beta / 2;

    dexp +=
        half_beta * w.Ceil() + (1.0 - alpha) / (wnorm * wnorm) * w.CeilSqr();
  }
}

void dExpInv_SO3(Eigen::Matrix<Scalar, 3, 3> &dexpinv) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  dexpinv.setIdentity();

  if (wnorm > RMATH_ZERO) {
    double half_wnorm = wnorm / 2;
    double s = std::sin(half_wnorm) / half_wnorm;
    double c = std::cos(half_wnorm);

    double gamma = c / s;
    double wnorm_sqr = wnorm * wnorm;

    // Eigen::Matrix3d temp1 = -0.5*w.Ceil(); temp1.print("-1/2*w.Ceil()");
    // Eigen::Matrix3d temp2 = (1 - gamma)/wnorm_sqr*w.CeilSqr();
    // temp2.print("(1 - gamma)/wnorm_sqr*w.CeilSqr()");

    dexpinv += -0.5 * w.Ceil() + (1 - gamma) / wnorm_sqr * w.CeilSqr();
  }
}

void ddt_dExp_SO3(const Eigen::Matrix<Scalar, 3, 1> &wdot,
                  Eigen::Matrix<Scalar, 3, 3> &ddt_dexp) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  if (wnorm > RMATH_ZERO) {
    double half_wnorm = wnorm / 2;
    double s = std::sin(half_wnorm) / half_wnorm;

    double beta = s * s;
    double half_beta = beta / 2;

    // ddt_dexp = half_beta*wdot.Ceil();

    double c = std::cos(half_wnorm);

    double alpha = s * c;
    double gamma = c / s;

    double wnorm_sqr = wnorm * wnorm;
    double temp = (1.0 - alpha) / wnorm_sqr;
    double w_wdot = w.dot(wdot);

    ddt_dexp = half_beta * wdot.Ceil() +
               (alpha - beta) / wnorm_sqr * w_wdot * w.Ceil() +
               temp * w.Ceil(wdot) +
               (half_beta - 3 * temp) * w_wdot / wnorm_sqr * w.CeilSqr();
  } else {
    ddt_dexp = 0.5 * wdot.Ceil();
  }
}

void ddt_dExpInv_SO3(const Eigen::Matrix<Scalar, 3, 1> &wdot,
                     Eigen::Matrix<Scalar, 3, 3> &ddt_dexpinv) {
  const Vector3d &w = *this;

  double wnorm = w.norm();

  ddt_dexpinv = -0.5 * wdot.Ceil();

  if (wnorm > RMATH_ZERO) {
    double half_wnorm = wnorm / 2;
    double s = std::sin(half_wnorm) / half_wnorm;
    double c = std::cos(half_wnorm);

    double alpha = s * c;
    double beta = s * s;
    double gamma = c / s;
    double half_beta = beta / 2;

    double wnorm_sqr = wnorm * wnorm;
    double w_wdot = w.dot(wdot);

    ddt_dexpinv +=
        (1 - gamma) / wnorm_sqr * w.Ceil(wdot) +
        (1 / beta + gamma - 2) / wnorm_sqr * w_wdot / wnorm_sqr * w.CeilSqr();
  }
}
