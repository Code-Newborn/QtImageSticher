#include "surfMethod.h"
#include <mainwindow.h>

four_corners_t corners_surf;

void CalcCorners(const cv::Mat &H, const cv::Mat &src)
{
    double v2[] = {0, 0, 1};                  // 左上角
    double v1[3];                             // 变换后的坐标值
    cv::Mat V2 = cv::Mat(3, 1, CV_64FC1, v2); // 列向量
    cv::Mat V1 = cv::Mat(3, 1, CV_64FC1, v1); // 列向量

    V1 = H * V2;
    // 左上角(0,0,1)
    std::cout << "V2: " << V2 << std::endl;
    std::cout << "V1: " << V1 << std::endl;
    corners_surf.left_top.x = v1[0] / v1[2];
    corners_surf.left_top.y = v1[1] / v1[2];

    // 左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2); // 列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1); // 列向量
    V1 = H * V2;
    corners_surf.left_bottom.x = v1[0] / v1[2];
    corners_surf.left_bottom.y = v1[1] / v1[2];

    // 右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2); // 列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1); // 列向量
    V1 = H * V2;
    corners_surf.right_top.x = v1[0] / v1[2];
    corners_surf.right_top.y = v1[1] / v1[2];

    // 右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2); // 列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1); // 列向量
    V1 = H * V2;
    corners_surf.right_bottom.x = v1[0] / v1[2];
    corners_surf.right_bottom.y = v1[1] / v1[2];
}

void OptimizeSeam(cv::Mat &img1, cv::Mat &trans, cv::Mat &dst)
{
    int start = MIN(corners_surf.left_top.x, corners_surf.left_bottom.x); // 开始位置，即重叠区域的左边界

    double processWidth = img1.cols - start; // 重叠区域的宽度
    int rows = dst.rows;
    int cols = img1.cols; // 注意，是列数*通道数
    double alpha = 1;     // img1中像素的权重
    for (int i = 0; i < rows; i++)
    {
        uchar *p = img1.ptr<uchar>(i); // 获取第i行的首地址
        uchar *t = trans.ptr<uchar>(i);
        uchar *d = dst.ptr<uchar>(i);
        for (int j = start; j < cols; j++)
        {
            // 如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                // img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
                alpha = (processWidth - (j - start)) / processWidth;
            }

            d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
        }
    }
}

void SurfStich(cv::Mat &img1, cv::Mat &img2, cv::Mat &result)
{
    // 灰度图转换
    cv::Mat image1, image2;
    cv::cvtColor(img1, image1, cv::COLOR_RGB2GRAY);
    cv::cvtColor(img2, image2, cv::COLOR_RGB2GRAY);

    // 提取特征点
    int minHessian = 2000;
    cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> Detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
    std::vector<cv::KeyPoint> keyPoint1, keyPoint2;
    Detector->detect(image1, keyPoint1);
    Detector->detect(image2, keyPoint2);

    // 特征点描述，为下边的特征点匹配做准备
    cv::Ptr<cv::xfeatures2d::SurfDescriptorExtractor> Descriptor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    cv::Mat imageDesc1, imageDesc2;
    Descriptor->compute(image1, keyPoint1, imageDesc1);
    Descriptor->compute(image2, keyPoint2, imageDesc2);

    // Fast Library for Approximate Nearest Neighbors【FLANN】快速近似最近邻搜索算法 匹配器
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matchePoints;
    std::vector<cv::DMatch> GoodMatchePoints;
    std::vector<cv::Mat> train_desc(1, imageDesc1);
    matcher.add(train_desc);
    matcher.train();
    matcher.knnMatch(imageDesc2, matchePoints, 2);
    std::cout << "total match points: " << matchePoints.size() << std::endl;

    // Lowe's algorithm,获取优秀匹配点
    for (int i = 0; i < matchePoints.size(); i++)
    {
        if (matchePoints[i][0].distance < 0.6 * matchePoints[i][1].distance)
        {
            GoodMatchePoints.push_back(matchePoints[i][0]);
        }
    }
    cv::Mat first_match;
    cv::drawMatches(img2, keyPoint2, img1, keyPoint1, GoodMatchePoints, first_match);
    cv::imwrite("first_match.jpg", first_match);

    std::vector<cv::Point2f> imagePoints1, imagePoints2;
    for (int i = 0; i < GoodMatchePoints.size(); i++)
    {
        imagePoints2.push_back(keyPoint2[GoodMatchePoints[i].queryIdx].pt);
        imagePoints1.push_back(keyPoint1[GoodMatchePoints[i].trainIdx].pt);
    }

    // 获取图像2到图像1的投影映射矩阵 尺寸为3*3
    cv::Mat homo = cv::findHomography(imagePoints1, imagePoints2, cv::RANSAC); // 计算多个二维点对之间的最优单映射变换矩阵 H（3行x3列） ，使用最小均方误差或者RANSAC方法
    //// 也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差
    // Mat homo = getPerspectiveTransform(imagePoints1, imagePoints2);
    std::cout << "变换矩阵为：\n"

              << homo << std::endl
              << std::endl; // 输出映射矩阵

    // 计算配准图的四个顶点坐标
    CalcCorners(homo, img1);
    std::cout << "left_top:" << corners_surf.left_top << std::endl;
    std::cout << "left_bottom:" << corners_surf.left_bottom << std::endl;
    std::cout << "right_top:" << corners_surf.right_top << std::endl;
    std::cout << "right_bottom:" << corners_surf.right_bottom << std::endl;

    // 图像配准
    cv::Mat imageTransform;
    cv::warpPerspective(img1, imageTransform, homo, cv::Size(MAX(corners_surf.right_top.x, corners_surf.right_bottom.x), img2.rows)); // warpPerspective图像会有裁剪
    // warpPerspective(image01, imageTransform2, adjustMat*homo, Size(image02.cols*1.3, image02.rows*1.8));
    cv::imwrite("imageTransform.jpg", imageTransform);

    // 创建拼接后的图,需提前计算图的大小
    int dst_width = imageTransform.cols; // 取最右点的长度为拼接图的长度
    int dst_height = img2.rows;

    cv::Mat dst(dst_height, dst_width, CV_8UC3);
    dst.setTo(0);

    imageTransform.copyTo(dst(cv::Rect(0, 0, imageTransform.cols, imageTransform.rows)));
    img2.copyTo(dst(cv::Rect(0, 0, img2.cols, img2.rows)));

    // cv::imshow("b_dst", dst);

    OptimizeSeam(img2, imageTransform, dst);
    result = dst;
    cv::imwrite("dst.jpg", dst);
}
