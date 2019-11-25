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
// 工程自定义的头文件 weihao123456
#include <sceneRepresentation.h>
#include <stereoFrame.h>
#include <stereoFrameHandler.h>
#include "dataset.h"
#include "timer.h"
// boost文件系统
#include <boost/filesystem.hpp>
using namespace StVO;

void showHelp();
bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file);

int main(int argc, char **argv)
{

    // read inputs
    // 从命令行中读取输入的配置信息
    string dataset_name, config_file;//数据集名称和配置文件名称
    int frame_offset = 0, frame_number = 0, frame_step = 1;//数据集前方丢弃帧,frame_step
    if (!getInputArgs(argc, argv, dataset_name, frame_offset, frame_number, frame_step, config_file)) {
        showHelp();
        return -1;
    }
    //配置文件不为空(判断命令行配置文件参数是否为空)则新建Config类
    if (!config_file.empty()) Config::loadFromFile(config_file);

    // read dataset root dir fron environment variable
    // 读取~/.bashrc文件中定义的数据集位置参数 DATASETS_DIR
    boost::filesystem::path dataset_path(string( getenv("DATASETS_DIR")));
    if (!boost::filesystem::exists(dataset_path) || !boost::filesystem::is_directory(dataset_path)) {
        cout << "Check your DATASETS_DIR environment variable" << endl;
        return -1;
    }

    dataset_path /= dataset_name;//dataset_path + '/' + dataset_name
    if (!boost::filesystem::exists(dataset_path) || !boost::filesystem::is_directory(dataset_path)) {
        cout << "Invalid dataset path" << endl;
        return -1;
    }

    //得到了数据集的完整路径, 类似:/home/weihao/Datastes/MH_01_easy/mav0
    string dataset_dir = dataset_path.string();
    //定义针孔双目相机模型类(用相机参数文件进行初始化)
    PinholeStereoCamera*  cam_pin = new PinholeStereoCamera((dataset_path / "dataset_params.yaml").string());
    // 定义数据集类: 初始化参数: 数据集路径, 相机模型, frame_offset, 帧数, frame_step
    Dataset dataset(dataset_dir, *cam_pin, frame_offset, frame_number, frame_step);//变量前加'*'表示求指针的值,变量前加'&'表示求变量的地址

    // create scene
    Matrix4d Tcw, T_inc = Matrix4d::Identity();
    Vector6d cov_eig;
    Matrix6d cov;
    Tcw = Matrix4d::Identity();
    Tcw << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;//世界坐标系到相机坐标系的变换矩阵

    // 创建场景(显示窗口)
    sceneRepresentation scene("../config/scene_config.ini");
    scene.initializeScene(Tcw, true);//初始化只利用了位姿

    Timer timer;

    // initialize and run PL-StVO
    int frame_counter = 0; //帧数统计
    double t1;
    // 定义双目相机VO
    StereoFrameHandler* StVO = new StereoFrameHandler(cam_pin);
    Mat img_l, img_r;
    while (dataset.nextFrame(img_l, img_r)) //从数据集读取了一帧图像(2张)
    {
        //初始化 VO
        if( frame_counter == 0 ) // initialize
            StVO->initialize(img_l,img_r,0);
        else // run
        {
            // PL-StVO
            // VO正常状态
            timer.start();
            StVO->insertStereoPair( img_l, img_r, frame_counter );//插入新的一帧
            StVO->optimizePose();//位姿优化
            t1 = timer.stop();

            // 读取当前帧的位姿参数
            T_inc   = StVO->curr_frame->DT;
            cov     = StVO->curr_frame->DT_cov;
            cov_eig = StVO->curr_frame->DT_cov_eig;

            // update scene
            // 场景参数设置与场景更新
            //scene.setText(frame_counter,t1,StVO->n_inliers_pt,StVO->matched_pt.size(),StVO->n_inliers_ls,StVO->matched_ls.size());
           // scene.setCov( cov );
            scene.setPose( T_inc );
            scene.setImage(StVO->curr_frame->plotStereoFrame());//显示图像
            //scene.updateScene(StVO->matched_pt, StVO->matched_ls);//显示的点和线的来源是matched_pt
            scene.updateScene();

            // console output
            // 输出结果
            /**
             * c++知识补充:
             * cout.setf      :通过设置格式标志来控制输出形式
             * ios::fixed     :用正常的记数方法显示浮点数(与科学计数法相对应)
             * ios::floatfield:表示小数点后保留6位小数
             * cout.precision :表示小数点后保留8位小数
             * \t             :表示转义符(空格)
             */
            cout.setf(ios::fixed,ios::floatfield); cout.precision(8); //设置输出格式
            cout << "Frame: " << frame_counter << "\tRes.: " << StVO->curr_frame->err_norm;
            cout.setf(ios::fixed,ios::floatfield); cout.precision(3);//输出精度为3
            cout << " \t Proc. time: " << t1 << " ms\t ";
            if( Config::adaptativeFAST() )  cout << "\t FAST: "   << StVO->orb_fast_th;//orb-fast阈值?
            if( Config::hasPoints())        cout << "\t Points: " << StVO->matched_pt.size() << " (" << StVO->n_inliers_pt << ") " ;//点的匹配数和内点数
            if( Config::hasLines() )        cout << "\t Lines:  " << StVO->matched_ls.size() << " (" << StVO->n_inliers_ls << ") " ;//线的匹配数和内点数
            cout << endl;

            // update StVO
            StVO->updateFrame();//更新V0
        }

        frame_counter++;//帧数+1
    }

    // wait until the scene is closed
    // 窗口不关闭就一直等待
    while( scene.isOpen() );

    return 0;
}
// 输出帮助信息
void showHelp() {
    cout << endl << "Usage: ./imagesStVO <dataset_name> [options]" << endl
         << "Options:" << endl
         << "\t-c Config file" << endl //配置文件
         << "\t-o Offset (number of frames to skip in the dataset directory" << endl //数据集前面跳过100帧
         << "\t-n Number of frames to process the sequence" << endl //总共处理1000帧图像
         << "\t-s Parameter to skip s-1 frames (default 1)" << endl //数据集中每两帧取一帧?
         << endl;
}
// 从命令行中读取输入的参数
bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file) {

    if( argc < 2 || argc > 10 || (argc % 2) == 1 ) //检查输入的命令是否正确,最少2项,最多10,而且是偶数
        return false;

    dataset_name = argv[1];//输入的第二个参数是数据集的名称
    int nargs = argc/2 - 1;//全部输入argc=10,nargs=4(从argv[2]开始算起)
    for( int i = 0; i < nargs; i++ )
    {
        int j = 2*i + 2;
        if( string(argv[j]) == "-o" )
            frame_offset = stoi(argv[j+1]);// stoi 表示读取string类型表示的数
        else if( string(argv[j]) == "-n" )
            frame_number = stoi(argv[j+1]);
        else if( string(argv[j]) == "-s" )
            frame_step = stoi(argv[j+1]);
        else if (string(argv[j]) == "-c")
            config_file = string(argv[j+1]);
        else
            return false;
    }

    return true;
}
