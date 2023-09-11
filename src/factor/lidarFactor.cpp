// Lidar residual factor, based on Ceres library
// This code refers to
// ALOAM: https://github.com/HKUST-Aerial-Robotics/A-LOAM
// Waynee: https://zhuanlan.zhihu.com/p/348195351
# include "lidarFactor.hpp"
Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in){
    Eigen::Matrix<double,3,3> skew_mat;
    skew_mat.setZero();
    skew_mat(0,1) = -mat_in(2);
    skew_mat(0,2) =  mat_in(1);
    skew_mat(1,2) = -mat_in(0);
    skew_mat(1,0) =  mat_in(2);
    skew_mat(2,0) = -mat_in(1);
    skew_mat(2,1) =  mat_in(0);
    return skew_mat;
}
void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t){
    Eigen::Vector3d omega(se3.data());
    Eigen::Vector3d upsilon(se3.data()+3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor = cos(half_theta);
    if(theta<1e-10)
    {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
    }
    else
    {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
    }

    q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());


    Eigen::Matrix3d J;
    if (theta<1e-10)
    {
        J = q.matrix();
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }

    t = J*upsilon;
}
bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

    quater_plus = delta_q * quater;
    trans_plus = delta_q * trans + delta_t;

    return true;
}
bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    (j.bottomRows(1)).setZero();

    return true;
}

// gp point to pg point, one dimensional residual
bool LidarPointFactorDirX::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{
    //Quaternions and translations
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    //std::cout<<"q_last_curr "<<q_last_curr.coeffs()<< " t_last_curr "<<t_last_curr<<" "<<std::endl;
    Eigen::Matrix<double, 1, 3> e ;
    e << 1,0,0;
    //apply transformation
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; //new point

    //compute residual
    residuals[0] = e* s*(lp - last_point);
    //std::cout<<"residuals "<<residuals[0]<<std::endl;

    //Jacobian calculation, jacobians are two-dimensional pointers, there are several parameter blocks,
    // corresponding to several Jacobians
    if(jacobians != nullptr)
    {
        if(jacobians[0] != nullptr)
        {
            //3*6 matrix, [row 0-2, column 0-2] stores the Skew-symmetric matrix, [row 0-2, column 3-5] stores
            // the identity matrix,
            Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_lp;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();

            //J_se3 Jacobian
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) =  e * dp_by_so3;
        }
    }
    return true;
}
bool LidarPointFactorDirY::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{
    //Quaternions and translations
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Matrix<double, 1, 3> e (0,1,0);
    //apply transformation
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; //new point

    //compute residual
    residuals[0] = e*s*(lp - last_point);;

    //compute the Jacobian matrix
    if(jacobians != nullptr)
    {
        if(jacobians[0] != nullptr)
        {
            Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_lp;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();

            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = e * dp_by_so3;
        }
    }
    return true;
}
bool LidarPointFactorDirZ::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{
    //Quaternions and translations
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Matrix<double, 1, 3> e (0,0,1);
    //apply transformation
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; //new point

    //compute residual
    residuals[0] = e*s*(lp - last_point);;

    //compute the Jacobian matrix
    if(jacobians != nullptr)
    {
        if(jacobians[0] != nullptr)
        {
            Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_lp;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();

            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = e * dp_by_so3;
        }
    }
    return true;
}

//point to plane. plane consist of three points
/*SurfAnalyticCostFunction::SurfAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                                                   Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_)
        : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),last_point_m(last_point_m_) {
    //cross product to compute the normal of plane
    ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
    ljm_norm.normalize();
}
bool SurfAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{
    //transformation between two frames
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    //apply transformation
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr;

    //compute residual
    residuals[0] = (lp - last_point_j).dot(ljm_norm);

    //compute the Jacobian matrix
    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_point_w = skew(lp);

            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_point_w;
            (dp_by_so3.block<3,3>(0,3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = ljm_norm.transpose() * dp_by_so3;

        }
    }
    return true;

}*/

//point to plane residual, given normal
SurfNormAnalyticCostFunction::SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_,
                                                           Eigen::Vector3d plane_unit_norm_, double weight_)
        : curr_point(curr_point_), last_point(last_point_), plane_unit_norm(plane_unit_norm_), weight(weight_) {
}
bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);

    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;

    residuals[0] = weight*plane_unit_norm.dot(point_w - last_point);

    if(jacobians != nullptr)
    {
        if(jacobians[0] != nullptr)
        {
            Eigen::Matrix3d skew_point_w = skew(point_w);

            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_point_w;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = weight*plane_unit_norm.transpose() * dp_by_so3;

        }
    }
    return true;

}

