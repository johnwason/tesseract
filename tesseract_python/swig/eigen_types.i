%eigen_typemaps(Eigen::Vector2d);
%eigen_typemaps(Eigen::Vector3d);
%eigen_typemaps(Eigen::Vector4d);
%eigen_typemaps(Eigen::Isometry3d);
%eigen_typemaps(Eigen::VectorXd);
%eigen_typemaps(Eigen::MatrixXd);
%eigen_typemaps(%arg(Eigen::Matrix3Xd));
%eigen_typemaps(%arg(Eigen::Matrix<uint32_t,3,Eigen::Dynamic>));
%eigen_typemaps(%arg(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>));
//Workaround typemaps for Isometry3d

%typemap(in, fragment="Eigen_Fragments") Eigen::Isometry3d &, Eigen::Isometry3d const& (Eigen::Isometry3d temp)
{
  // In: plain, non-const&, const&
  Eigen::Matrix4d temp_matrix;  
  if (!ConvertFromNumpyToEigenMatrix<Eigen::Matrix4d>(&temp_matrix, $input))
    SWIG_fail;
  temp = temp_matrix;
  $1 = &temp;
}
%typemap(in, fragment="Eigen_Fragments") Eigen::Isometry3d (Eigen::Isometry3d temp)
{
  // In: plain, non-const&, const&
  Eigen::Matrix4d temp_matrix;  
  if (!ConvertFromNumpyToEigenMatrix<Eigen::Matrix4d>(&temp_matrix, $input))
    SWIG_fail;
  temp = temp_matrix;
  $1 = temp;
}
%typemap(out, fragment="Eigen_Fragments") Eigen::Isometry3d, Eigen::Isometry3d const
{
  Eigen::Matrix4d temp_matrix=$1.matrix();
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Matrix4d >(&$result, &temp_matrix))
    SWIG_fail;
}
%typemap(out, fragment="Eigen_Fragments") Eigen::Isometry3d&, Eigen::Isometry3d const&
{
  Eigen::Matrix4d temp_matrix=$1->matrix();
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Matrix4d >(&$result, &temp_matrix))
    SWIG_fail;
}