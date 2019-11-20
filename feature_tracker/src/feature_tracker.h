#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"


#include "parameters.h"
// tic_toc.h中是作者自己封装的一个类TIC_TOC，用来计时
#include "tic_toc.h"


using namespace std;
using namespace camodocal;
using namespace Eigen;

// 判断跟踪的特征点是否在图像边界内
bool inBorder(const cv::Point2f &pt);

// 去除无法跟踪的特征点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);


class FeatureTracker
{
  public:
    FeatureTracker();

    // 对新来的图像使用光流法进行特征点跟踪
    void readImage(const cv::Mat &_img,double _cur_time);

    // 对跟踪点进行排序并去除密集点
    void setMask();

    // 添加新检测到的特征点n_pts，ID初始化-1，跟踪次数1
    void addPoints();

    // 更新特征点id
    bool updateID(unsigned int i);

    // 读取相机内参
    void readIntrinsicParameter(const string &calib_file);

    // 显示去畸变矫正后的特征点
    void showUndistortion(const string &name);

    // 通过基本矩阵（F）去除外点outliers
    void rejectWithF();

    // 对特征点的图像坐标去畸变矫正，并计算每个角点的速度
    void undistortedPoints();

    cv::Mat mask;                 // 图像掩码
    cv::Mat fisheye_mask;         // 鱼眼相机mask，用来去除边缘噪点
 
    // prev_img  上一次发布的帧的图像数据
    // cur_img   光流跟踪的前一帧的图像数据
    // forw_img  光流跟踪的后一帧的图像数据
    cv::Mat prev_img, cur_img, forw_img;

    vector<cv::Point2f> n_pts;                          // 每一帧中新提取的特征点
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;    // 对应的图像特征点
    vector<cv::Point2f> prev_un_pts, cur_un_pts;        // 归一化相机坐标系下的坐标
    vector<cv::Point2f> pts_velocity;                   // 当前帧相对前一帧特征点沿x,y方向的像素移动速度
    vector<int> ids;            // 能够被跟踪到的特征点的id
    vector<int> track_cnt;      // 当前帧forw_img中每个特征点被追踪的时间次数

    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;

    camodocal::CameraPtr m_camera;      //相机模型

    double cur_time;
    double prev_time;

    // 用来作为特征点id，每检测到一个新的特征点，就将n_id作为该特征点的id，然后n_id加1
    static int n_id;
};
