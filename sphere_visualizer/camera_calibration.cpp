
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


void createTargetPoints(cv::Size boardSize, cv::Size2f itemSize, std::vector<cv::Point3f>& points)
{
    for(int i = 0; i<boardSize.height; i++)
    {
        for(int j = 0; j<boardSize.width; j++)
        {
            points.push_back(cv::Point3f(j*itemSize.width, i*itemSize.height, 0));
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 3)
        return -1;

    cv::VideoCapture cap(std::stoi(argv[1]), cv::CAP_V4L2);
    if(!cap.isOpened())
    {
        return -2;
    }

    cv::Size frameSize(1280, 780);
    cv::Size boardSize(11,7);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameSize.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameSize.height);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

    frameSize.width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    frameSize.height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    std::vector<cv::Point3f> objectPoints;
    createTargetPoints(boardSize, cv::Size2f(2,2), objectPoints);


    
    cv::Mat frame;
    cv::Mat gray;
    char key = 0;

    std::vector< std::vector< cv::Point3f > > objectsPoints;
    std::vector< std::vector< cv::Point2f > > imgPoints;
    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 5, 0.001);

    while(cap.read(frame) && key != 27)
    {
        std::vector<cv::Point2f> corners;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(gray, boardSize, corners);
        if(found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1), criteria);
            cv::drawChessboardCorners(frame, boardSize, corners, found);
        }
        cv::imshow("capture", frame);
        key = cv::waitKey(10);
        if(key==32)
        {
            if(found)
            {
                imgPoints.push_back(corners);
                objectsPoints.push_back(objectPoints);
            }
        }
    }

    cv::destroyAllWindows();
    cap.release();

    if(!imgPoints.empty())
    {
        cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;
        
        cv::calibrateCamera(objectsPoints, imgPoints, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        cv::FileStorage calibFile(argv[2],cv::FileStorage::WRITE);

        calibFile.write("cameraWidth", frameSize.width);
        calibFile.write("cameraHeight", frameSize.height);
        calibFile.write("cameraMatrix", cameraMatrix);
        calibFile.write("distortionCoeffs", distCoeffs);

        calibFile.release();

    }

    

    return 0;
}