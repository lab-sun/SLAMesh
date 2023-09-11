#pragma once
#ifndef POINTMATRIX_H_
#define POINTMATRIX_H_
//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <map>

typedef Eigen::Matrix4d Transf;
typedef Eigen::Matrix<double, 3, 1> Point;
typedef Eigen::Matrix<double, 4, 1> PointWithVariance;
typedef Eigen::Matrix<double, 6, 1> State;//x y z roll pitch yaw, angle in rad

class PointMatrix{
    //3d points container based on Eigen, because I want more flexible functions
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double stamp = -1;//timestamp
    int num_point = 0;
    const int resize_step = 64;// when size of point matrix is not enough, resize it
    const int init_size = 32;//512 256
    Eigen::Matrix<double, 3, Eigen::Dynamic> point;
    Eigen::Matrix<double, 1, Eigen::Dynamic> variance;
    Point centroid{0,0,0};
    Eigen::Matrix3d eigenvectorMat, eigenvalMat;
    Eigen::Vector3d min_vector{0,0,0};//or normal vector
    double min_value = -1;
    double variance_sum = -1;
    std::map<double, Eigen::Matrix<double, 3, 1>> eig_sorted;

    PointMatrix(){
        //initialize
        variance = Eigen::MatrixXd::Zero(1, init_size);
        point = Eigen::MatrixXd::Zero(3, init_size);
        variance.fill(100);//100 is a very large value, means not usable
    }
    explicit PointMatrix(int size){
        //initialize with given size
        variance = Eigen::MatrixXd::Zero(1, size);
        point = Eigen::MatrixXd::Zero(3, size);
        variance.fill(100);
    }
    explicit PointMatrix(double _stamp){
        //initialize with given timestamp
        variance = Eigen::MatrixXd::Zero(1, init_size);
        point = Eigen::MatrixXd::Zero(3, init_size);
        stamp = _stamp;
        variance.fill(100);
    }
    void clear_quick(){
        //clear data by changing num_point, save time
        num_point = 0;
        stamp = -1;
        centroid.fill(0);
        eigenvectorMat.fill(0);
        eigenvalMat.fill(0);
        min_vector.fill(0);
        eig_sorted.clear();
        min_value = -1;
        variance_sum = -1;
    }
    void clear(){
        //clear all data
        point.fill(0);
        variance.fill(100);
        clear_quick();
    }
    PointMatrix(const PointMatrix & points_copy){
        //deep copy function
        if(points_copy.num_point > point.cols()){
            point.resize(3, points_copy.num_point);
            variance.resize(1, points_copy.num_point);
        }
        clear();

        num_point = points_copy.num_point;
        stamp = points_copy.stamp;
        centroid = points_copy.centroid;
        eigenvectorMat = points_copy.eigenvectorMat;
        eigenvalMat = points_copy.eigenvalMat;
        min_vector = points_copy.min_vector;
        min_value = points_copy.min_value;
        variance_sum = points_copy.variance_sum;
        point.leftCols(points_copy.num_point) = points_copy.point.leftCols(points_copy.num_point);
        variance.leftCols(points_copy.num_point) = points_copy.variance.leftCols(points_copy.num_point);
    }
    PointMatrix & operator = (const PointMatrix & points_copy){
        //deep copy function use = operator
        if(points_copy.num_point > point.cols()){
            point.resize(3, points_copy.num_point);
            variance.resize(1, points_copy.num_point);
        }
        clear();

        num_point = points_copy.num_point;
        stamp = points_copy.stamp;
        centroid = points_copy.centroid;
        eigenvectorMat = points_copy.eigenvectorMat;
        eigenvalMat = points_copy.eigenvalMat;
        min_vector = points_copy.min_vector;
        min_value = points_copy.min_value;
        variance_sum = points_copy.variance_sum;
        point.leftCols(points_copy.num_point) = points_copy.point.leftCols(points_copy.num_point);
        variance.leftCols(points_copy.num_point) = points_copy.variance.leftCols(points_copy.num_point);
        return *this;
    }
    PointMatrix & operator = (const Eigen::MatrixXd & points_copy){
        //use Eigen::MatrixXd to initialize
        if(points_copy.cols() > point.cols()){
            point.resize(3, points_copy.cols());
            variance.resize(1, points_copy.cols());
        }
        clear();

        point.leftCols(points_copy.cols()) = points_copy.topRows(3);
        num_point = int(points_copy.cols());
        if(points_copy.rows() == 4){
            variance.leftCols(points_copy.cols()) = points_copy.row(3);
        }
        else{
            variance.leftCols(points_copy.cols()).fill(100);
        }
        return *this;
    }
    PointMatrix & operator += (const PointMatrix & points_add){
        //add points by a PointMatrix
        if(points_add.num_point == 0){
            return *this;
        }
        int space_left = int(point.cols()) - (points_add.num_point + num_point);
        if(space_left < -1*resize_step){
            point.conservativeResize(Eigen::NoChange_t(3), points_add.num_point + num_point + resize_step);
            variance.conservativeResize(Eigen::NoChange_t(1), points_add.num_point + num_point + resize_step);
        }
        else if(space_left < 0){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            variance.conservativeResize(Eigen::NoChange_t(1), point.cols() + resize_step);
            point.rightCols(resize_step).fill(0);
            variance.rightCols(resize_step).fill(100);
        }
        point.middleCols(num_point, points_add.num_point) = points_add.point.leftCols(points_add.num_point);
        variance.middleCols(num_point, points_add.num_point) = points_add.variance.leftCols(points_add.num_point);
        num_point += points_add.num_point;
        return *this;
    }
    void addPoint(const Eigen::Matrix<double, 3, 1> & point_add){
        //add a point
        if( num_point+1 > point.cols()){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            variance.conservativeResize(Eigen::NoChange_t(1), point.cols() + resize_step);
            point.rightCols(resize_step).fill(0);
            variance.rightCols(resize_step).fill(100);
        }
        point.col(num_point) = point_add;
        variance(0, num_point) = 100;
        num_point++;
    }
    void addPointWithVariance(const Eigen::Matrix<double, 4, 1> & point_add_with_variance){
        //add a point with variance
        if( num_point+1 > point.cols()){
            point.conservativeResize(Eigen::NoChange_t(3), point.cols()+resize_step);
            variance.conservativeResize(Eigen::NoChange_t(1), point.cols() + resize_step);
            point.rightCols(resize_step).fill(0);
            variance.rightCols(resize_step).fill(100);
        }
        point.col(num_point) = point_add_with_variance.topRows(3);
        variance(0, num_point)   = point_add_with_variance(3, 0);
        num_point++;
    }
    PointWithVariance getPointWithVariance(int index){
        //get a point with variance
        if(index > num_point){
            std::cout << "error, out of range. " <<std::endl;
        }
        Eigen::Matrix<double, 4, 1> result_point;
        result_point.topRows(3) = point.col(index);
        result_point(3,0) = variance(0, index);
        return result_point;
    }
    void print(){
        std::cout << "timestamp: " << stamp  << " num_point: " << num_point << " xyz: " <<  std::endl;
        std::cout << point.leftCols(num_point) <<  std::endl;
        if(variance(0, 0) != 100){
            std::cout << "variance: " << std::endl << variance.leftCols(num_point) << std::endl;
        }
        std::cout<<"size of matrix: \n"<<point.cols()<<std::endl;
    }
    void printVariance(){
        std::cout << "num_point: " << num_point << " variance: " <<
                  " max " << variance.leftCols(num_point).maxCoeff() << " min " << variance.leftCols(num_point).minCoeff() << std::endl;
        double count_variance[10];
        for(double i : count_variance){
            i = 0;
        }
        for(int i = 0; i < num_point; i++){
            count_variance[int(variance(i) * 10)]++;
        }
        for(double i : count_variance){
            i=i/num_point*100;
            std::cout << int(i) <<"%";
        }
        std::cout<<std::endl;
    }
    PointMatrix copyAndTransform(Transf & transf){
        //copy a PointMatrix and transform it
        PointMatrix points_transformed(*this);//copy, so we can return a new PointMatrix
        Eigen::Matrix<double, 4, Eigen::Dynamic> tmp_point_expend;
        tmp_point_expend = Eigen::MatrixXd::Zero(4, 1).replicate(1, num_point);
        tmp_point_expend.topRows(3) = points_transformed.point.leftCols(num_point);
        tmp_point_expend.bottomRows(1).fill(1);
        tmp_point_expend = transf * tmp_point_expend;
        points_transformed.point.leftCols(num_point) = tmp_point_expend.topRows(3);
        return points_transformed;
    }
    void transformPoints(Transf & trans){
        //transform points
        if(trans(0, 0) == 0 && trans(1, 0) == 0 && trans(2, 0) == 0){
            std::cout<<"ERROR: Wrong Trans!";
        }

        Eigen::Matrix<double, 4, Eigen::Dynamic> tmp_point_expend;
        tmp_point_expend = Eigen::MatrixXd::Zero(4, num_point);
        tmp_point_expend.topRows(3) = point.leftCols(num_point);
        tmp_point_expend.bottomRows(1).fill(1);
        tmp_point_expend = trans * tmp_point_expend;
        point.leftCols(num_point) = tmp_point_expend.topRows(3);
    }
    void eigenDecomposition(){
        //Eigen decomposition
        if(num_point != 0){
            centroid = point.leftCols(num_point).rowwise().sum() / num_point;

            Eigen::Matrix<double, 3, Eigen::Dynamic> remove_centroid =
                    point.leftCols(num_point) - centroid.replicate(1, num_point);
            Eigen::Matrix3d covarianceMat = remove_centroid * remove_centroid.adjoint() / num_point;
            Eigen::EigenSolver<Eigen::Matrix3d> eig(covarianceMat);
            eigenvalMat = eig.pseudoEigenvalueMatrix();
            eigenvectorMat = eig.pseudoEigenvectors();

            for(int i = 0; i < 3; i ++){
                eig_sorted.emplace(eigenvalMat(i, i), eigenvectorMat.col(i));
            }
            //for(auto & i_map : eig_sorted){
                //std::cout<<"eig result: "<<i_map.first<<std::endl;//<<i_map.second<<std::endl;
            //}
            min_value  = (eigenvalMat(0, 0) < eigenvalMat(1, 1)) ?
                    ((eigenvalMat(0, 0) < eigenvalMat(2, 2)) ? (eigenvalMat(0, 0)) : (eigenvalMat(2, 2))) :
                    ((eigenvalMat(1, 1) < eigenvalMat(2, 2)) ? (eigenvalMat(1, 1)) : (eigenvalMat(2, 2)));
            //std::cout<<"min_value old: "<<min_value<<std::endl;
            min_vector = (eigenvalMat(0, 0) < eigenvalMat(1, 1)) ?
                    ((eigenvalMat(0, 0) < eigenvalMat(2, 2)) ? (eigenvectorMat.col(0)) : (eigenvectorMat.col(2))) :
                    ((eigenvalMat(1, 1) < eigenvalMat(2, 2)) ? (eigenvectorMat.col(1)) : (eigenvectorMat.col(2)));
        }else{
           std::cout<< "eigenDecomposition error: no point"<<std::endl;
       }
    }
};

#endif


