// 实现一个类FeatureTracker，用来完成特征点提取和特征点跟踪等主要功能；
#include "feature_tracker.h"

int FeatureTracker::n_id = 0;


// 判断跟踪的特征点是否在图像边界内
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}


// 去除无法追踪的特征点(根据传入的参数类型，选择不同的函数进行运算)
// 将　status　判断为0 的点去掉，即没有匹配上的点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}


// 对图像使用光流法进行特征点跟踪
// 按照被追踪到的次数排序，然后加上mask去掉部分点(密集点)，主要是针对鱼眼相机
/* 通过设置一个mask，使跟踪的特征点在整幅图像中能够均匀分布，防止特征点扎堆。
FeatureTracker::setMask()的具体操作为：
对光流跟踪到的特征点forw_pts，按照被跟踪到的次数降序排列，然后按照降序遍历这些特征点。
每选中一个特征点，在mask中将该点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的特征点。
这样会删去一些特征点，使得特征点分布得更加均匀，同时尽可能地保留被跟踪次数更多的特征点。
未被mask的地方会重新检测然后提取新的特征点进来？？？*/

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));     // mask 初始化为255
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    // 对图像中提取的forw_pts中的点排序　按照track_cnt[i]由大到小排序。 sort(begin, end, compare)
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        //　at<uchar>　当该点的灰度值为255时，取出图像中的点。
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);                // 将满足上述条件的点push到forw_pts 中
            ids.push_back(it.second.second);                    // 保存对应的id
            track_cnt.push_back(it.first);                      // 保存对应对应forw_pts 在　cnt_pts_id 里面的索引
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1); // mask　一定范围内设为０
        }
    }
}


// 添加新检测到的特征点n_pts，ID初始化-1，跟踪次数1
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

// void FeatureTracker::readImage()     对新来的图像使用光流法进行特征点跟踪
/* 处理流程为:
1. 先调用cv::CLAHE对图像做直方图均衡化(如果EQUALIZE=1，表示太亮或则太暗)
2. 调用calcOpticalFlowPyrLK()跟踪cur_pts到forw_pts,
   根据status,把跟踪失败的点剔除(注意:prev, cur,forw, ids, track_cnt都要剔除)
   这里还加了个inBorder判断,把跟踪到图像边缘的点也剔除掉
3. 如果不需要发布特征点,则到这步就完了,把当前帧forw赋给上一帧cur, 然后退出.
   如果需要发布特征点(PUB_THIS_FRAME=1), 则执行下面的步骤
4. 先调用rejectWithF()对prev_pts和forw_pts做ransac剔除outlier.(实际就是调用了findFundamentalMat函数)
   在光流追踪成功就记被追踪+1，数值代表被追踪的次数，数值越大，说明被追踪的就越久
5. 调用setMask(), 先对跟踪点forw_pts按跟踪次数降排序, 然后依次选点
   选一个点, 在mask中将该点周围一定半径的区域设为0, 后面不再选取该区域内的点
   有点类似与non-max suppression, 但区别是这里保留track_cnt最高的点
6. 在mask中不为0的区域,调用goodFeaturesToTrack提取新的角点n_pts
   通过addPoints()函数push到forw_pts中, id初始化-1,track_cnt初始化为1
   
   整体来说需要注意的是：光流跟踪在 2 中完成，角点提取在 6 中完成*/

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)       // 判断均衡化的标志
    {
        // createCLAHE() 判断并对图像进行对比度受限的自适应直方图均衡化CLAHE
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

        TicToc t_c;     // 记录时间
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

     // readImage()，这里涉及到3个img(prev_img, cur_img, forw_img)和pts(prev_pts,cur_pts, forw_pts)，两者是相似的。
     // cur和forw分别是LK光流跟踪的前后两帧，forw才是真正的“当前”帧，
     // cur实际上是上一帧，而prev是上一次发布的帧，它实际上是光流跟踪以后，prev和forw根据Fundamental Matrix做RANSAC剔除outlier用的，也就是rejectWithF()函数
    if (forw_img.empty())   // 判断容器是否为空
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    // 在已提取了上一帧特征点的情况下进入
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        // 从cur_pts到forw_pts做LK金字塔光流法，参数分别为：
        // cur_img:     输入 上一帧，即标定图像的灰度图
        // frow_img:    输入 当前帧，想搜寻的图像灰度图
        // cur_pts:     输入 上一帧特征点，输入标定图像特征点
        // frow_pts:    输出 当前跟踪特征点，输出场景的特征点
        // status：     输出状态向量，uchar，如果在当前图像中能够光流得到标定的特征点位置改变，则设置status为1，否则设置为0
        // error：      输出错误向量；向量的每个元素被设为相应特征的一个错误，float型，包含原始图像碎片与一动点之间的差
        // cv::Size();  金字塔所有尺寸，
        // maxLevel;    最大金字塔层数,默认设置为3
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            // 判断光流匹配的是否在图像边界内,（１< border < row/col -1）
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        // 以上所有reduceVector 将点或者索引的向量中的没有匹配的点或者索引去掉

        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;

    
/*  由于光流跟踪到的特征点会减少，而且setMask()的处理过程中也会删除一些特征点，所以需要新检测一些特征点。
    （只有需要发布数据时，才会检测新的特征点，否则只跟踪，不检测新的特征点）。
    具体操作为：调用cv::goodFeaturesToTrack()在mask中不为0的区域检测新的特征点，将特征点数量补充至指定数量。
    然后调用FeatureTracker::addPoints()，将新检测到的特征点到forw_pts中去，id初始化为-1，track_cnt初始化为1。*/

/* void cv::goodFeaturesToTrack(
        cv::InputArray image,               // 输入图像（CV_8UC1 CV_32FC1）
        cv::OutputArray corners,            // 输出角点vector
        int maxCorners,                     // 最大角点数目
        double qualityLevel,                // 质量水平系数（小于1.0的正数，一般在0.01-0.1之间）
        double minDistance,                 // 最小距离，小于此距离的点忽略
        cv::InputArray mask = noArray(),    // mask=0的点忽略
        int blockSize = 3,                  // 使用的邻域数
        bool useHarrisDetector = false,     // false ='Shi Tomasi metric' true = Harris"
        double k = 0.04                     // Harris角点检测时使用
    );
*/
    if (PUB_THIS_FRAME)
    {
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;

        // MAX_CNT 是最大光流特征点数量
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if(mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if(mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;
}


// 通过前后两帧的追踪计算F矩阵，通过F矩阵去除Outliers
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}


// 更新特征点ID
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}


// 读取相机内参:根据不同的相机模型，实例化不同camera,并设置参数
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}


// 显示去畸变矫正后的特征点(图像去畸变)
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}


// 对特征点的图像坐标去畸变矫正，并计算每个角点的速度
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
