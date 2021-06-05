#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>
#include <math.h>

const int height = 600;
const int width = height;
int cenY = height/2;
int cenX = width/2;
const int graphHeight = 300;
const int graphmargin = (height-graphHeight)/2;
const int expanRate = 15;

class LiderVis
{
ros::NodeHandle nh;
image_transport::ImageTransport it;
ros::Subscriber vecSub;
image_transport::Publisher graphImgPub;

private:
    double maxRangeVal;
    double minRangeVal;
    double minAngle;
    double maxAngle;
    double dRad;


    cv::Mat image;
    std::vector<float> laserData;
    template <class T> T clip(const T& n, float lower, float upper);
    template <class T> T normalize(const T& n, float xmin, float xmax ,float amin, float amax);
    float minValue(const std::vector<float>& value);
    float maxValue(const std::vector<float>& value);
    void lasor_callback(const sensor_msgs::LaserScan& laser);
public:
    LiderVis();
    ~LiderVis();
};

LiderVis::LiderVis():it(nh)
{
    ros::NodeHandle pnh("~");
    pnh.getParam("maxRange", maxRangeVal);

    graphImgPub = it.advertise("scanImage", 10);
    vecSub = nh.subscribe("scan", 10, &LiderVis::lasor_callback, this);
}

LiderVis::~LiderVis()
{
    //cv::destroyAllWindows();
}


template <class T> T LiderVis::clip(const T& n, float lower, float upper)
{
    T numbers = n;
    for(auto& number : numbers)
    {
        number = std::max(lower, std::min(number, upper));
    }
  return numbers;
}

template <class T> T LiderVis::normalize(const T& n, float xmin, float xmax ,float amin, float amax)
{
    T numbers = n;
    for(auto& number : numbers){
        number = (amax - amin) * (number - xmin) / (xmax - xmin) + amin;
    }
    return numbers;
}

float LiderVis::minValue(const std::vector<float>& value)
{
    float minV = 99999999;

    for(int i=0; i<value.size(); i++){
        if(minV>value[i]){
            minV = value[i];
        }
    }

    return minV;

}

float LiderVis::maxValue(const std::vector<float>& value)
{
    float maxV = -99999999;

    for(int i=0; i<value.size(); i++){
        if(maxV<value[i]){
            maxV = value[i];
        }
    }

    return maxV;

}

void LiderVis::lasor_callback(const sensor_msgs::LaserScan& laser)
{
    minRangeVal = laser.range_min;
    maxRangeVal = laser.range_max;
    minAngle = laser.angle_min;
    maxAngle = laser.angle_max;
    dRad = laser.angle_increment;
    double rad = M_PI_2 - minAngle;

    laserData = normalize(laser.ranges, minRangeVal, maxValue(laser.ranges), 5, graphHeight/2);

    image = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    //center dots
    cv::circle(image, cv::Point(cenX, cenY), 2, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
    //laser
    for(int i=0; i<laser.ranges.size(); i++){
        int x = laserData[i] * cos(rad) + cenX;
        int y = -laserData[i] * sin(rad) + cenY;
        //draw red dots
        cv::circle(image, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        /*image.at<cv::Vec3b>(y,x)[0] = 0;
        image.at<cv::Vec3b>(y,x)[1] = 0;
        image.at<cv::Vec3b>(y,x)[2] = 255;*/
        rad += dRad;
    }

    //cv::putText(image, std::to_string(minValue(vecData_row)), cv::Point(25,height-15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2);

    sensor_msgs::ImagePtr scanImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    graphImgPub.publish(scanImage);

    //image_g = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8)->image;
    //cv::imshow("image", image);
    //cv::waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "LiderVisualizer_2d");
    LiderVis lv;

    ros::spin();
    return 0;
}