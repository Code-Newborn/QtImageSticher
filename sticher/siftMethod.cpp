#include "siftMethod.h"
#include <mainwindow.h>

four_corners_t corners_sift;

void SiftStich(cv::Mat &img1, cv::Mat &img2, cv::Mat &result)
{

    cv::resize(img1, img1, img1.size());
    cv::resize(img2, img2, img1.size());
    // 灰度图转换
    cv::Mat image1, image2;
    cv::cvtColor(img1, image1, cv::COLOR_RGB2GRAY);
    cv::cvtColor(img2, image2, cv::COLOR_RGB2GRAY);

        // 提取特征点
    int numfeatures = 2000;
    cv::Ptr<cv::SiftFeatureDetector> Detector = cv::SiftFeatureDetector::create(numfeatures);
    std::vector<cv::KeyPoint> keyPoint1, keyPoint2;
    Detector->detect(image1, keyPoint1);
    Detector->detect(image2, keyPoint2);

    // 特征点描述，为下边的特征点匹配做准备
    cv::Ptr<cv::SiftDescriptorExtractor> Descriptor = cv::SiftDescriptorExtractor::create();
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
    std::cout << "Transform Matrix：\n"

              << homo << std::endl
              << std::endl; // 输出映射矩阵

    // 计算配准图的四个顶点坐标
    CalcCorners(homo, img1);
    std::cout << "left_top:" << corners_sift.left_top << std::endl;
    std::cout << "left_bottom:" << corners_sift.left_bottom << std::endl;
    std::cout << "right_top:" << corners_sift.right_top << std::endl;
    std::cout << "right_bottom:" << corners_sift.right_bottom << std::endl;

    // 图像配准
    cv::Mat imageTransform;
    cv::warpPerspective(img1, imageTransform, homo, cv::Size(MAX(corners_sift.right_top.x, corners_sift.right_bottom.x), img2.rows)); // warpPerspective图像会有裁剪
    // warpPerspective(image01, imageTransform2, adjustMat*homo, Size(image02.cols*1.3, image02.rows*1.8));
    cv::imwrite("imageTransform.jpg", imageTransform);

    // 创建拼接后的图,需提前计算图的大小
    int dst_width = imageTransform.cols; // 取最右点的长度为拼接图的长度
    int dst_height = img2.rows;

    cv::Mat dst(dst_height, dst_width, CV_8UC3);
    dst.setTo(0);

    imageTransform.copyTo(dst(cv::Rect(0, 0, imageTransform.cols, imageTransform.rows)));
    img2.copyTo(dst(cv::Rect(0, 0, img2.cols, img2.rows)));

    cv::imwrite("b_dst.jpg", dst);

    OptimizeSeam(img2, imageTransform, dst);
    result = dst;
    cv::imwrite("dst.jpg", dst);
}