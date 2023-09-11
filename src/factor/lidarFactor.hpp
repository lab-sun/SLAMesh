#ifndef LIDAR_FACTOR_H
#define LIDAR_FACTOR_H
#endif //LIDAR_FACTOR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

//Skew-symmetric matrix declaration
Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in);
//Get quaternion q and translation t from transformation matrix se3 group
void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);
// used in computeTbyCeres, use AutoDiffCostFunction, so no Evaluate function
/*struct LidarPointFactor
{
    LidarPointFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_, double s_)
            : curr_point(curr_point_), last_point(last_point_), s(s_) {}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {

        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lp_obs{T(last_point.x()), T(last_point.y()), T(last_point.z())};

        //Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        //Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        //q_last_curr = q_identity.slerp(T(s), q_last_curr);
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp_vir;
        lp_vir = q_last_curr * cp + t_last_curr;

        residual[0] = s*(lp_vir.x() - lp_obs.x());
        residual[1] = s*(lp_vir.y() - lp_obs.y());
        residual[2] = s*(lp_vir.z() - lp_obs.z());

//        residual[0] = sqrt(pow(lp_vir.x() - lp_obs.x(), 2) +
//                           pow(lp_vir.y() - lp_obs.y(), 2) +
//                           pow(lp_vir.z() - lp_obs.z(), 2));

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_, const double s_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarPointFactor, 3, 4, 3>(
                new LidarPointFactor(curr_point_, last_point_, s_)));
    }

    Eigen::Vector3d curr_point, last_point;
    double s;
};*/
// used in computeT, use AutoDiffCostFunction
/*struct LidarPointFactorDir
{
    LidarPointFactorDir(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_, double s_, int dir_)
            : curr_point(curr_point_), last_point(last_point_), s(s_), dir(dir_){}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {

        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lp_obs{T(last_point.x()), T(last_point.y()), T(last_point.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp_vir;
        lp_vir = q_last_curr * cp + t_last_curr;

        residual[0] = s*(lp_vir(dir, 0) - lp_obs(dir, 0));

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_, const double s_, const int dir_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarPointFactorDir, 1, 4, 3>(
                new LidarPointFactorDir(curr_point_, last_point_, s_, dir_)));
    }

    Eigen::Vector3d curr_point, last_point;
    double s;
    int dir;

};*/

// gp point to pg point, one dimensional residual, give  jacobians
class LidarPointFactorDirX : public ceres::SizedCostFunction<1, 7>
{
public:
    LidarPointFactorDirX(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_, double s_)
            : curr_point(curr_point_), last_point(last_point_), s(s_){}

    virtual ~LidarPointFactorDirX() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point, last_point;
    double s;
};
class LidarPointFactorDirY : public ceres::SizedCostFunction<1, 7>
{
public:
    LidarPointFactorDirY(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_, double s_)
            : curr_point(curr_point_), last_point(last_point_), s(s_){}

    virtual ~LidarPointFactorDirY() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point, last_point;
    double s;
};
class LidarPointFactorDirZ : public ceres::SizedCostFunction<1, 7>
{
public:
    LidarPointFactorDirZ(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_, double s_)
            : curr_point(curr_point_), last_point(last_point_), s(s_){}

    virtual ~LidarPointFactorDirZ() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point, last_point;
    double s;
};

//point to plane. plane consist of three points
/*class SurfAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
public:
    SurfAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_);
    virtual ~SurfAnalyticCostFunction() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
};*/

//point to plane residual, given normal
class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
public:
    SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_, Eigen::Vector3d plane_unit_norm_, double weight_);
    virtual ~SurfNormAnalyticCostFunction() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point;
    Eigen::Vector3d last_point;
    Eigen::Vector3d plane_unit_norm;
    double weight;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
    PoseSE3Parameterization() = default;
    virtual ~PoseSE3Parameterization() {}

    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};

