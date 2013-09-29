#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const char WINDOW[] = "Image window";

class ImageConverter {
public:
    ImageConverter() : it(nh) {
        image_sub = it.subscribe("input_image", 1, &ImageConverter::onNewImageReceived, this);
        image_pub = it.advertise("processed_image", 1);
        cv::namedWindow(WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(WINDOW);
    }

    void onNewImageReceived(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr rawImagePtr;
        try {
            rawImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv_bridge::CvImage processedImage;
        processedImage.image = rawImagePtr->image;
        
        // Process the image here
        
        cv::imshow(WINDOW, processedImage.image);
        cv::waitKey(3);
        image_pub.publish(processedImage.toImageMsg());
    }
    
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
