#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

Estimator estimator;

std::condition_variable con;
double current_time = -1;

queue<sensor_msgs::ImuConstPtr> imu_buf;                // 先进先出
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;

int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;   // 
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;      // 前一个IMU加速度计数据
Eigen::Vector3d gyr_0;      // 前一个IMU陀螺仪数据
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;


// 系统入口
int main(int argc, char **argv)
{
    // 1 ROS初始化、设置句柄
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    // 2 读取参数，设置状态估计器参数
    readParameters(n);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    // 3 发布用于rviz显示的Topic(visualization.cpp)
    registerPub(n);

    // 4 订阅几个话题，队列长度为2000，将imu，feature，raw_image数据（用于回环检测）通过各自的回调函数封装起来
    /* create a templated subscriber
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > (topic, queue_size, callback);*/

    //   订阅 IMU话题，调用 imu_callback() 将新得到的IMU数据放入到 imu_buf 队列中
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());

    //   订阅 img话题，调用 feature_callback() 将最新的特征点数据存入 feature_buf 队列中
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    
    //   订阅 restart话题，
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    
    //   订阅 match_points话题，
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    // 5 开启VIO主线程，处理measurement
    std::thread measurement_process{process};
    
    ros::spin();

    return 0;
}



// IMU预积分，从IMU测量值imu_msg和上一个PVQ递推得到当前PVQ
// sensor_msgs::ImuConstPtr  IMU信息的标准数据结构 from file：sensor_msgs/imu.msg 
// imu_msg[采样时间内单次IMU测量值]
void predict(const sensor_msgs::ImuConstPtr &imu_msg)   
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};    // 线加速度

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};       // 角速度

    // 代码采用的是基于中值法的离散形式而不是论文中展示的欧拉法
    // 这个地方的tmp_Q是local-->world ？？？
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    // 对应式(2),
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    // 预积分的过程中Bias没有发生改变

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// 得到窗口最后一个图像帧的imu项[P,Q,V,ba,bg,a,g]，对imu_buf中剩余imu_msg进行PVQ递推
// （因为imu的频率比图像频率要高很多，在getMeasurements(）将图像和imu时间对齐后，imu_buf中还会存在imu数据）
void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

// 对imu和图像数据进行初步对齐，使得一副图像对应多组imu数据，并确保相邻图像对应时间戳内的所有IMU数据
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> getMeasurements()
{
    // 定义观测值数据类型 measurements 包含一组IMU数据和一帧图像数据的组合容器
    // sensor_msgs::ImuConstPtr 表示当前帧和上一帧时间间隔中的所有IMU数据
    // sensor_msgs::PointCloudConstPtr 表示某一帧图像的feature_points
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        // 直到把缓存中的图像特征数据或者IMU数据取完，才能够跳出此函数
        // 保证存在IMU数据和图像特征数据
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        // 对齐标准：IMU最后一个数据的时间要大于第一个图像特征数据的时间
        // 判断图像特征数据和IMU数据是否对齐 如果最新的IMU的数据时间戳小于最旧特征点的时间戳，则等待IMU刷新
        // 满足两个条件就能保证数据对齐，满足数据对齐就可以把数据从队列中按对齐的方式取出来。
        // 第一是IMU最后一个数据的时间要大于图像特征最开始数据的时间，第二是IMU最开始数据的时间要小于图像特征最开始数据的时间。
        // 把缓存中的图像特征数据或者IMU数据取完，才能够跳出此函数，返回数据。
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        // 对齐标准：IMU第一个数据的时间要小于第一个图像特征数据的时间
        // 如果最旧的IMU数据的时间戳大于最旧特征时间戳，则弹出旧图像
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        // 这里IMU和Feature做了简单的对齐，确保IMU的时间戳是小于图像的
        // 在IMU buff中的时间戳小于特征点的都和该帧特征联合存入
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        
        //  图像数据(img_msg)，对应多组在时间戳内的imu数据,然后塞入measurements
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        
        // 这里把下一个imu_msg也放进去了,但没有pop，因此当前图像帧和下一图像帧会共用这个imu_msg
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

// imu回调函数，将imu_msg存入imu_buf，递推IMU的PQV并发布"imu_propagate”
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();           // 互斥线程锁，在这个时段内只允许一个线程进入执行，其它线程等待
    imu_buf.push(imu_msg);  // 将数据存到imu_buf
    m_buf.unlock();
    con.notify_one();       // 唤醒线程 作用于process线程中获取观测值数据的函数

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        // C++11锁管理器 std::lock_guard 调用即加锁，离开大括号作用域自动解锁
        std::lock_guard<std::mutex> lg(m_state);
        
        // 这个地方积分是为了提高系统位姿的输出频率
        // 预测未考虑观测噪声的p v q 值 
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            // 发布最新的IMU测量值消息 pvq 这里计算得到的是pvq估计值 是没有观测噪声和bias的结果 
            // 作用是与下面预积分得到的pvq(考虑了观测噪声和bias)做差得到残差
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);     // "imu_propagate"
    }
}


// 将特征数据 feature_msg 保存到 feature_buf
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

// restart回调函数，收到restart消息时清空feature_buf和imu_buf，估计器重置，时间重置
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// relocalization回调函数，将points_msg放入relo_buf
void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}


// thread: visual-inertial odometry  处理观测数据
// 注意这里只保证了相邻的feature数据之间有完整的imu数据，并不能保证imu和feature数据的精确对齐
// 首先将获取的传感器数据imu_buf feature_buf对齐

void process()
{
    // 通过while (true)不断循环，主要功能包括等待并获取measurements，计算dt，然后执行以下函数：
    //      stimator.processIMU()       进行IMU预积分
    //      estimator.setReloFrame()    设置重定位帧
    //      estimator.processImage()    处理图像帧：初始化，紧耦合的非线性优化
    while (true)
    {
        // 1. 获取Features和IMU测量数据 measurements 
        //      存储单帧图像对应多帧IMU数据的结构
        //      使用互斥锁和条件等待 
        //      互斥锁用来锁住当前代码段，条件等待是等待上面两个接受数据完成就会被唤醒，然后从buf中提取观测数据
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
                    // 从imu_buf和feature_buf中提取观测数据
                    return (measurements = getMeasurements()).size() != 0;
                 });
        
        // 提取观测值数据的时候互斥锁会锁住imu_buf和feature_buf等提取完成才释放，在提取的过程中上面两个回调函数是无法接收数据的
        // 同时上面两个回调函数接收数据的时候也使用了互斥锁，锁住imu_buf和feature_buf，这里也不能提取imu_buf和feature_buf中的数据
        // 数据获取过程：回调函数接收数据，接收完一组数据唤醒提取数据的线程，提取数据的线程提取完数据后，回调函数就可以继续接收数据，依次往复。
        lk.unlock();
        m_estimator.lock();
        
        // 一个measurement,包含很多个imu_msg和一个img_msg数据,imu数据的时间戳都是小于img_msg的
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            
            // 2. 分别取出各段imu数据，进行预积分
            for (auto &imu_msg : measurement.first)
            {
                // 发送imu数据进行预积分 if中代码对应其它版本中的send_imu
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    
                    // 对于measurement中的每一个imu_msg，计算dt并执行processIMU()。
                    // processIMU()实现了IMU的预积分，通过中值积分得到当前PQV作为优化初值
                    // dt为当前IMU数据和前一个IMU数据的时间差,(dx, dy, dz)为当前加速度计数据,(rx, ry, rz)为当前角速度计数据
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            
            // 3. set relocalization frame
            //  在relo_buf中取出最后一个重定位帧，拿出其中的信息并执行setReloFrame()
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                
                // channels[0] point_id，
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            // 建立每个特征点的(camera_id,[x,y,z,u,v,vx,vy])s的map，索引为feature_id
            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                // 归一化平面上的点
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1); // 判断是否归一化了 
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            
            // 处理图像，这里实现了视觉与IMU的初始化以及非线性优化的紧耦合
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            // 给RVIZ发送里程计信息PQV、关键点三维坐标、相机位姿、点云信息、IMU到相机的外参、重定位位姿等
            pubOdometry(estimator, header);     // "odometry"
            pubKeyPoses(estimator, header);     // "key_poses"
            pubCameraPose(estimator, header);   // "camera_pose"
            pubPointCloud(estimator, header);   // "history_cloud"
            pubTF(estimator, header);           // "extrinsic"
            pubKeyframe(estimator);             // "keyframe_point"、"keyframe_pose"
            if (relo_msg != NULL)
                pubRelocalization(estimator);   // "relo_relative_pose"
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();   // 更新IMU参数[P,Q,V,ba,bg,a,g]
        m_state.unlock();
        m_buf.unlock();
    }
}

