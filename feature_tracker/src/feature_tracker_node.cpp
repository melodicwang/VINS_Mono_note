// 构造一个ROS节点feature_tracker_node，主要调用FeatureTracker类来实现前端功能。
/*** 特征点跟踪的具体实现  feature_trackers可被认为是一个单独的模块 ***/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

// 发布的全局变量(话题)
ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

// NUM_OF_CAM为相机的个数，这意味这每一个相机都有一个FeatureTracker的实例
// 区分双目和单目的数组 由FeatureTracker类的实例组成的数组
FeatureTracker trackerData[NUM_OF_CAM];

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;


/* void img_callback(const sensor_msgs::ImageConstPtr &img_msg)    ***/
/* 
 * ROS的图像回调函数，主要功能包括：
 *      readImage()函数对新来的图像使用光流法进行特征点跟踪，
 *      将追踪的特征点封装成feature_points,发布到pub_img
 *      将图像封装到cv_bridge::cvtColor类型的ptr实例中发布到pub_match
 * 
 * 1. 判断是否是第一帧
 * 2. 判断时间间隔是否正确，有问题则restart
 * 3. 发布频率控制，并不是每读入一帧图像，就要发布特征点，通过判断间隔时间内的发布次数
 * 4. 将图像编码8UC1转换为mono8
 * 5. 单目时：FeatureTracker::readImage() 函数读取图像数据进行处理
 * 6. 更新全局ID
 * 7. 如果PUB_THIS_FRAME=1则进行发布
 *      将特征点id，矫正后归一化平面的3D点(x,y,z=1)，像素2D点(u,v)，像素的速度(vx,vy)，
 *      封装成sensor_msgs::PointCloudPtr类型的feature_points实例中,发布到pub_img;
 */

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 1. 判断是否是第一帧，不是跳过
    if(first_image_flag)
    {
        // 如果是第一帧，则将flag置false，避免下一次调用的时候再进来
        first_image_flag = false;

        // 计算第一帧图像产生的时间 .toSec用来对齐时间
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }

    // detect unstable camera stream
    // 2. 判断时间间隔是否正确，正确跳过，有问题则restart
    // 逻辑或运算
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        
        // 程序错误，重启，将发布次数重新置为初始值1
        pub_count = 1;

        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }

    // 将当前帧的时间赋值给last_image_time,这样就一直保持last_image_time是最新帧的时间
    last_image_time = img_msg->header.stamp.toSec();

    // 3. 发布频率控制，通过判断间隔时间内的发布次数来决定是否发布
    //    并不是每读入一帧图像，就要发布特征点，
    //    不发布的时候也是会执行readImage()读取图像进行处理，将检测到的feature作为下一时刻的KLT追踪的特征点

    // 小于或等于FREQ的时候就进入 发布；
    // double round(doube x);  把一个小数四舍五入
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        // 发布
        PUB_THIS_FRAME = true;
     
        // reset the frequency control  
        // 时间间隔内的发布频率十分接近设定频率时，重置频率控制
        // 更新时间间隔起始时刻，并将数据发布次数置0
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    // 4. 将图像编码8UC1转换为mono8，ROS图像与OpenCV图像类型进行转换
    //    ROS转OpenCV还是?
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    // 5. 单目时：FeatureTracker::readImage() 函数读取图像数据进行处理
    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)    
    {
        // ？
        ROS_DEBUG("processing camera %d", i);
        
        /*  单目处理逻辑
            如果是单目相机（双目开关STEREO_TRACK为0），则只有一个相机：相机0。
            调用FeatureTracker::readImage()函数，读取单目图像数据，
            然后在readImage()函数中，对前一帧图像中的特征点进行金字塔光流跟踪，必要时检测新的特征点对特征点数量进行补充。*/
        if (i != 1 || !STEREO_TRACK)
            // 单目时  调用FeatureTracker的readImage()  用光流跟踪上一帧
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        // 双目处理逻辑
        // 如果是双目相机（双目开关STEREO_TRACK为1），则有两个相机：相机0和相机1。对于相机0：在readImage()函数中，前后两帧图像之间进行金字塔光流跟踪，必要时在当前帧中检测新特征点以补充特征点数量。对于相机1：如果需要发布当前帧的数据（PUB_THIS_FRAME为true），且相机0的前一帧图像中特征点数量不为空，则直接在回调函数img_callback()中，相机1的当前帧图像对相机0的前一帧图像进行金字塔光流跟踪，这里光流跟踪的处理过程与单目模式下的类似，只是不会补充新的特征点；否则不需要进一步处理。
        else
        {
            if (EQUALIZE)   // 执行均衡化(图像太亮或太黑)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();  // 直方图均衡化
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else            // 左右两帧分别放在trackerData的数组中[0],[1] 中
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
    }


    // 6. 更新全局ID
    /*    特征点id相当于特征点的身份证号，对数据关联（data association）至关重要。
    需要注意的是，更新特征点id的步骤被特意放到了回调函数img_callback()中，而不是FeatureTracker::readImage()函数内部。
    有一种说法是，n_id是FeatureTracker类的静态变量。*/


    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }


    /* 7. 如果PUB_THIS_FRAME=1则进行发布
        将特征点id，矫正后归一化平面的3D点(x,y,z=1)，像素2D点(u,v)，像素的速度(vx,vy)，
        封装成sensor_msgs::PointCloudPtr类型的feature_points实例中,发布到pub_img;
        将图像封装到cv_bridge::cvtColor类型的ptr实例中发布到pub_match      */
   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}



/*** 
int main()
    1、ros初始化和设置句柄，设置logger级别
    2、读取如config->euroc->euroc_config.yaml中的一些配置参数
    3、读取每个相机实例读取对应的相机内参，NUM_OF_CAM=1为单目
    4、判断是否加入鱼眼mask来去除边缘噪声
    5、订阅话题IMAGE_TOPIC(如/cam0/image_raw)，执行回调函数img_callback
    6、输出

    输入：图像，即订阅了传感器或者rosbag发布的topic：“/cam0/image_raw”
    输出：
        1、发布topic：“/feature_trackers/feature_img”
        即跟踪的特征点图像，主要是之后给RVIZ用和调试用

        2、发布topic：“/feature_trackers/feature”
        即跟踪的特征点信息，由/vins_estimator订阅并进行优化

        3、发布topic：“/feature_trackers/restart”
        即判断特征跟踪模块是否出错，若有问题则进行复位，由/vins_estimator订阅

其算法主要包括以下内容：
    1、对于每一幅新图像，KLT稀疏光流算法对现有特征进行跟踪；
    2、检测新的角点特征以保证每个图像特征的最小数目(100-300)；
    3、通过设置两个相邻特征之间像素的最小间隔来执行均匀的特征分布；
    4、利用基本矩阵模型的RANSAC算法进行外点剔除；
    5、对特征点进行去畸变矫正，然后投影到一个单位球面上(对于cata-fisheye camera)。    ******/

int main(int argc, char **argv)
{

    // "节点名"，需唯一
    ros::init(argc, argv, "feature_tracker");

    // 开始节点，n命名空间为/node_namespace/node_name
    // ~ 表示私有命名空间，以节点名称作为命名空间   
    ros::NodeHandle n("~");

    // ROS日志输出，编译时设置源代码的调试级别，调试的显示级别为Info    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // 通过ROS来读取如config->euroc->euroc_config.yaml中的一些配置参数
    // node需要有config_file参数来指定话题名称/相机类型/相机内参/cam-IMU外参/视觉特征追踪参数/后端优化参数/imu参数/回环检测的参数
    readParameters(n);

    // 读取每个相机实例对应的相机内参，NUM_OF_CAM=1为单目
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // 判断是否加入鱼眼mask来去除边缘噪声,追踪时候会用到
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // 订阅器sub_img从话题IMAGE_TOPIC中订阅相机图像数据，回调函数为img_callback()
    // 订阅话题IMAGE_TOPIC(如/cam0/image_raw)，执行回调函数img_callback,主要的操作都在回调函数中
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    // 发布feature_point / feature_img / restart_msg话题
    // 在feature话题下发布一条PointCloud类型的消息 队列长度为1000
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    // 在feature_img话题下发布一条Image类型的消息，实例ptr，跟踪的特征点图，给RVIZ用和调试用
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    // 在restart的话题下发布一条bool型的消息，判断特征跟踪模块是否出错，若有问题则进行复位，由/vins_estimator订阅
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

    // ROS消息回调处理函数,主程序到这儿就不往下执行了
    ros::spin();

    // 至此，已经将图像数据包装成特征点数据和图像数据发布出来了，下面就是再开一个线程，发布一个话题接收这两种消息，也就是vins_esitimator做的事
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
