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
    float maxRangeVal;
    float minRangeVal;

    cv::Mat image;
    std::vector<float> rangeData;
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

    graphImgPub = it.advertise("graphImage", 10);
    vecSub = nh.subscribe("sensorValue", 10, &LiderVis::lasor_callback, this);
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
    minRangeVal = -maxRangeVal;

    //->0~255
    rangeData = normalize(laser.ranges, minRangeVal, maxRangeVal, 0, graphHeight);

    image = cv::Mat(height, vecData_row.size()*(expanRate-1)+graphmargin*2, CV_8UC3, cv::Scalar(255, 255, 255));

    //centerLine
    cv::line(image, cv::Point(0, height/2), cv::Point(vecData_row.size()*(expanRate-1)+graphmargin*2, height/2), cv::Scalar(0,0,0), 3, 4);
    for(int i = 0;i<vecData_row.size()-1;i++){
        cv::line(image, cv::Point(i*expanRate+graphmargin, graphHeight-vecData[i]+graphmargin), cv::Point((i+1)*expanRate+graphmargin, graphHeight-vecData[i+1]+graphmargin), cv::Scalar(0,0,255), 2, 4);
    }

    cv::putText(image, std::to_string(maxValue(vecData_row)), cv::Point(25,15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2);
    cv::putText(image, std::to_string(minValue(vecData_row)), cv::Point(25,height-15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 2);

    sensor_msgs::ImagePtr graphImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    graphImgPub.publish(graphImage);

    //image_g = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::MONO8)->image;
    //cv::imshow("image", image);
    //cv::waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "2dLiderVis");
    LiderVis lv;

    ros::spin();
    return 0;
}