#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

std::vector<cv::Mat1d> generateSpherePoints(double radius, double inc)
{
    std::vector<cv::Mat1d> spherePoints;
    for (double ang1 = 0; ang1 < 2 * CV_PI; ang1 += inc)
    {
        for (double ang2 = 0; ang2 < 2 * CV_PI; ang2 += inc)
        {
            cv::Mat1d point(3, 1);
            point << radius * std::cos(ang2) * std::sin(ang1),
                radius * std::sin(ang2) * std::sin(ang1),
                radius * std::cos(ang1);

            spherePoints.push_back(point);
        }
    }
    return spherePoints;
}

int main(int argc, char **argv)
{

    cv::FileStorage file(argv[1], cv::FileStorage::READ);

    cv::Mat1d cameraMatrix, newCameraMatrix;
    cv::Mat1d distortionCoefficients;

    file["cameraMatrix"] >> cameraMatrix;
    newCameraMatrix = cameraMatrix.clone();
    file["distortionCoeffs"] >> distortionCoefficients;

    cv::Size frameSize;
    file["cameraWidth"] >> frameSize.width;
    file["cameraHeight"] >> frameSize.height;

    std::cout << cameraMatrix << std::endl
              << frameSize << std::endl
              << std::endl;

    cv::Mat map1, map2;

    cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    cv::initUndistortRectifyMap(cameraMatrix, distortionCoefficients, R, newCameraMatrix, frameSize, CV_32FC1, map1, map2);
    std::cout << newCameraMatrix << std::endl;

    cv::VideoCapture cap(std::atoi(argv[2]));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameSize.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameSize.height);

    cv::Mat frame, undistorted;

    char k = 0;

    cv::namedWindow("frame", cv::WINDOW_NORMAL);
    cv::namedWindow("undistorted frame", cv::WINDOW_NORMAL);

    cv::Mat imgWithSphere;

    std::vector<cv::Mat1d> spherePoints;
    cv::Mat1d center(3, 1);
    cv::Mat1d ref = cv::Mat1d::zeros(3, 1);
    center << 0, 0, 30;
    double radius = 1;

    spherePoints = generateSpherePoints(radius, 0.1);
    bool changed = false;
    double inc = 1;

    do
    {
        cap >> frame;
        // cv::GaussianBlur(frame, frame, cv::Size(5,5),1.8);
        cv::medianBlur(frame, frame, 3);
        // cv::undistort(frame, undistorted, cameraMatrix, distortionCoefficients);
        cv::remap(frame, undistorted, map1, map2, cv::INTER_AREA);
        cv::Rect imgView(0, 0, undistorted.cols, undistorted.rows);
        imgWithSphere = undistorted.clone();

        for (int i = 0; i < spherePoints.size(); i++)
        {
            cv::Mat1d result = cameraMatrix * (spherePoints[i] + center);

            result *= 1. / result.at<double>(2);
            cv::Point2i p;
            p.x = std::round(result.at<double>(0));
            p.y = std::round(result.at<double>(1));
            if (!imgView.contains(p))
                continue;
            cv::Vec3b &color = imgWithSphere.at<cv::Vec3b>(p);
            color[0] = 0;
            color[1] = 0;
            color[2] = 255;
        }

        cv::imshow("frame", frame);
        cv::imshow("undistorted frame", undistorted);
        cv::imshow("sphere", imgWithSphere);
        k = cv::waitKey(1);

        switch (k)
        {
        case 'q':
            center.at<double>(0) += inc;
            break;
        case 'a':
            center.at<double>(0) -= inc;
            break;

        case 'w':
            center.at<double>(1) += inc;
            break;
        case 's':
            center.at<double>(1) -= inc;
            break;

        case 'e':
            center.at<double>(2) += inc;
            break;
        case 'd':
            center.at<double>(2) -= inc;
            break;
        case 'r':
            radius += 1;
            break;
        case 'f':
            radius -= 1;
            break;
        case 'z':
            if (inc == 1)
            {
                inc = 0.1;
            }
            else
                inc = 1;
            break;
        case 32:

            std::cout << "Type the sphere parameters (X,Y,Z,R): " << std::endl;
            std::cin >> center.at<double>(0);
            std::cin >> center.at<double>(1);
            std::cin >> center.at<double>(2);
            std::cin >> radius;
        }
        if (k == 'q' || k == 'a' || k == 'w' || k == 's' || k == 'e' || k == 'd')
        {
            std::cout << undistorted.size() << std::endl;
            std::cout << center << std::endl;
            std::cout << cv::norm(center, ref, cv::NORM_L2) << std::endl
                      << std::endl;
        }
        if (k == 'r' || k == 'f')
        {
            std::cout << "Sphere Radius: " << radius << std::endl;
            spherePoints = generateSpherePoints(radius, 0.1);
        }

    } while (!frame.empty() && k != 27);

    return 0;
}
