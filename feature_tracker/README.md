**VINS-Mono视觉前端**

VINS-Mono采用LK光流法作为视觉前端，与前端相关的代码都在feature_tracker目录下。
（计算部分像素运动的称为稀疏光流，Lk即是；计算所有像素的称为稠密光流）

Feature Tracker其实仅仅做了视觉前端的数据关联，只是视觉前端的第一步。
最主要的流程就是通过订阅图像的Topic，获取图像后进行处理。处理得到了图像与图像之间的匹配关系，然后通过消息发布。

与ORB通过描述子进行关联匹配不同的是，VINS采用KLT光流对角点进行关联匹配，并通过id的方式记录关联的结果。
总的来说，速度快，抗干扰强，但牺牲了一定的精确度。

从实验效果上来看，追踪的成功率非常的高，虽然牺牲了一定的精确度，但不会像ORB-SLAM那样容易丢失。

```
sensor_msgs::PointCloud
{
	std_msgs/Header header
	geometry_msgs/Point32[] points			// points中存储了所有当前图像中追踪到的角点的图像归一化坐标。
	sensor_msgs/ChannelFloat32[] channels	// channels中存储了关于该角点的相关信息，这里共存储了5种信息。
											//	1.角点的id
											//	2.角点像素坐标的横坐标, 		3.角点像素坐标的纵坐标
											//	4.角点在x方向的速度,			5.角点在y方向的速度
}
```

sensor_msgs::PointCloud中所有的信息都是在void img_callback(const sensor_msgs::ImageConstPtr &img_msg)中处理得到的，实际依赖的是FeatureTracker类，


FeatureTracker类中几个比较重要的数据结构，如下：
```
camodocal::CameraPtr m_camera; 			// 相机模型，保存了相机的内参和相机投影方程
cv::Mat prev_img, cur_img, forw_img; 	// 原始图像数据。prev_img好像没啥用，cur_img保存了上一帧图像，forw_img保存了当前帧。
vector<cv::Point2f> prev_pts, cur_pts, forw_pts; // 图像中的角点坐标
vector<int> track_cnt; 		// 保存了当前追踪到的角点一共被多少帧图像追踪到
vector<int> ids; 			// 保存了当前追踪到的角点的ID，这个ID非常关键，保存了帧与帧之间角点的匹配关系。
```



**数据预处理**

*1、图像获取(feature_tracker节点)*

	a) 通过直方图均匀化的方法使得原本过亮或过暗的图像能够提取更多的特征点

	b) 用Harris角点为特征，采用KLT光流法对图像进行跟踪，从而实现特征的匹配
		这里光流法有一个特点：只跟踪前一帧已知的特征点，所以不会存在第1帧和第3帧有某个特征点，而第2帧没有这个特征点的情况。
		所以程序中的feature管理就很好的利用了这一点，去找每个特征点在每一个帧上出现的情况
	
	c)用PUB_THIS_FRAME作为标志，以10hz的频率向estimator节点发送点云内容，这里每一帧发送的图像包含足够多的特征点(150个角点)
	
	d)向estimator发布的点云内容包括：
		1)特征点在相机坐标系的归一化坐标;
		2)相机ID号;
		3)特征点的ID号;


*2、图像处理*

	a)图像处理的入口是void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)，是整个视觉前端最关键的代码。

		上一帧的图像和图像中的角点分别保存在cur_img和cur_pts中;
		利用cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3),
		函数可以得到当前图像中能通过光流追踪到的角点的坐标，保存在forw_pts中。

		由于前一帧保存的角点并不一定全部能够被当前帧追踪到，追踪的成功与否保存在status中。
		因此利用status和void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)函数对track_cnt，ids等进行更新。

		经过更新之后track_cnt里面保存都是能够追踪到的角点的追踪次数，执行+1操作。

	b)新角点的提取
		由于光流匹配会导致追踪到的角点越来与少，因此需要根据MAX_CNT和在当前帧中追踪到的角点数目差，进行新的角点的提取。
		依赖的函数为cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
		新提取的角点坐标保存在n_pts中。
		
		值得注意的是这里有个参数MIN_DIST，这个参数保证2个相邻角点之间的最小距离，通过这个参数可以在一定程度上保证角点的均匀分布。

	c)mask设置
		这里mask的设置也是有讲究的，因为在forw_img中已经通过光流追踪到了一部分角点，
		当再次从forw_img中提取角点的时候，通过设置相应的mask,来保证角点的提取不会重复。
		设置mask是通过void FeatureTracker::setMask()实现的。
		
		void FeatureTracker::setMask()里面有个很有趣的操作就是会根据角点被追踪的次数进行排序，
		即track_cnt, ids, forw_pts都是按照角点被追踪的次数排序的。
		
		之后会把新追踪到的角点n_pts加入到forw_pts和ids中去。
		
		最后调用void FeatureTracker::undistortedPoints()对角点图像坐标做去畸变处理，并计算每个角点的速度。

至此，所有sensor_msgs::PointCloud里面的数据都已经计算得到了。

