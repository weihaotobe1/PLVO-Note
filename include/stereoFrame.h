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

#include <future>
#include <thread>
#include <time.h>
#include <set>
#include <utility>
using namespace std;

#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
using namespace cv;
using namespace line_descriptor;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include <config.h>
#include <stereoFeatures.h>
#include <pinholeStereoCamera.h>
#include <auxiliar.h>

#define GRID_ROWS 48
#define GRID_COLS 64

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

namespace StVO{

class StereoFrame
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 构造函数和析构函数
    StereoFrame();
    StereoFrame(const Mat img_l_, const Mat img_r_, const int idx_, PinholeStereoCamera *cam_ );
    ~StereoFrame();

    //提取双目图片特征
    void extractStereoFeatures( double llength_th, int fast_th = 20 );
    void extractRGBDFeatures(   double llength_th, int fast_th = 20 );

    void detectStereoPoints(int fast_th = 20);
    // 提取一幅图像的orb特征
    void detectPointFeatures( Mat img, vector<KeyPoint> &points, Mat &pdesc, int fast_th = 20 );
    // 一帧图像的两幅图片特征匹配
    void matchStereoPoints(vector<KeyPoint> points_l, vector<KeyPoint> points_r, Mat &pdesc_l_, Mat pdesc_r, bool initial = false );
    //正常的特征匹配
    void matchPointFeatures(BFMatcher* bfm, Mat pdesc_1, Mat pdesc_2, vector<vector<DMatch>> &pmatches_12);

    // 一帧图像两幅图像的线段特征
    void detectStereoLineSegments(double llength_th);
    // 一幅图像的线段特征提取和描述子
    void detectLineFeatures( Mat img, vector<KeyLine> &lines, Mat &ldesc, double min_line_length );
    // 左右图像进行线段特征匹配
    void matchStereoLines(vector<KeyLine> lines_l, vector<KeyLine> lines_r, Mat &ldesc_l_, Mat ldesc_r, bool initial = false );
    // 正常的线段特征匹配
    void matchLineFeatures(BFMatcher* bfm, Mat ldesc_1, Mat ldesc_2, vector<vector<DMatch>> &lmatches_12 );
    // 过滤掉不一致的线段
    void filterLineSegmentDisparity(Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr , double &disp_s, double &disp_e);
    void filterLineSegmentDisparity(double &disp_s, double &disp_e);

    double lineSegmentOverlapStereo( double spl_obs, double epl_obs, double spl_proj, double epl_proj  );
    double lineSegmentOverlap( Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj  );
    void pointDescriptorMAD( const vector<vector<DMatch>> matches, double &nn_mad, double &nn12_mad );//orb特征
    void lineDescriptorMAD( const vector<vector<DMatch>> matches, double &nn_mad, double &nn12_mad );//线段特征

    Mat  plotStereoFrame();//显示两帧图像

    int frame_idx;
    Mat img_l, img_r;
    Matrix4d Tfw;//从世界坐标系到图像坐标系
    Matrix4d DT;

    Matrix6d Tfw_cov;//变换矩阵的协方差
    Vector6d Tfw_cov_eig;
    double   entropy_first;

    Matrix6d DT_cov;//DT??
    Vector6d DT_cov_eig;
    double   err_norm;

    vector< PointFeature* > stereo_pt;//点特征
    vector< LineFeature*  > stereo_ls;//线特征

    vector<KeyPoint> points_l, points_r;//关键点
    vector<KeyLine>  lines_l,  lines_r;//关键线
    Mat pdesc_l, pdesc_r, ldesc_l, ldesc_r;//左右两幅图像的描述子

    PinholeStereoCamera *cam;//相机参数

    double inv_width, inv_height; // grid cell(图像块的大小)
};

}
