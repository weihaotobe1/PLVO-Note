/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#pragma once
#include <stereoFrame.h>
#include <stereoFeatures.h>

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

class StereoFrame;

namespace StVO{

class StereoFrameHandler
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 构造函数和析构函数
    StereoFrameHandler( PinholeStereoCamera* cam_ );
    ~StereoFrameHandler();

    // VO初始化
    void initialize( const Mat img_l_, const Mat img_r_, const int idx_);
    void insertStereoPair(const Mat img_l_, const Mat img_r_, const int idx_);
    void updateFrame();//更新?

    void f2fTracking();//跟踪
    void matchF2FPoints();//ORB特征点匹配
    void matchF2FLines();//线特征匹配
    double f2fLineSegmentOverlap( Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj  );

    bool isGoodSolution( Matrix4d DT, Matrix6d DTcov, double err );
    void optimizePose();//位姿优化
    void resetOutliers();
    void setAsOutliers();

    void plotStereoFrameProjerr(Matrix4d DT, int iter);//显示投影?
    void plotLeftPair();

    // adaptative fast
    int orb_fast_th;
    double llength_th;

    // slam-specific functions
    bool needNewKF();
    void currFrameIsKF();

    //list< boost::shared_ptr<PointFeature> > matched_pt;
    //list< boost::shared_ptr<LineFeature>  > matched_ls;
    list< PointFeature* > matched_pt;//点特征
    list< LineFeature*  > matched_ls;//线特征

    StereoFrame* prev_frame;//前一帧图像
    StereoFrame* curr_frame;//当前帧图像
    PinholeStereoCamera *cam;//相机参数

    int  n_inliers, n_inliers_pt, n_inliers_ls;//内点个数,内线个数

    // slam-specific variables
    bool     prev_f_iskf;//前一帧是否为关键帧
    double   entropy_first_prevKF;
    Matrix4d T_prevKF;//上一关键帧的位姿
    Matrix6d cov_prevKF_currF;//当前帧和上一关键帧之间的协方差
    int      N_prevKF_currF;//当前帧和上一关键帧之间图像帧数

//    bool recurse;

private:

    void prefilterOutliers( Matrix4d DT );//预滤掉异常值
    void removeOutliers( Matrix4d DT );//去除异常值
    void gaussNewtonOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);//高斯牛顿算法进行优化
    void gaussNewtonOptimizationRobust(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);//带核函数的高斯牛顿算法
    void gaussNewtonOptimizationRobustDebug(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);
    void levenbergMarquardtOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);//LM算法
    void optimizeFunctions(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e);//优化函数
    void optimizeFunctionsRobust(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e);//鲁棒的优化函数
    void optimizePoseDebug();

};

}
