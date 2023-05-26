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
