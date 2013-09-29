#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class ImageConverter {
public:
    ImageConverter() : it(nh) {
        image_sub = it.subscribe("input_image", 1, &ImageConverter::onNewImageReceived, this);
        image_pub = it.advertise("processed_image", 1);
        cv::namedWindow("Raw image");
        cv::namedWindow("Processed image");
        if (!cascade.load("/home/john/uav-ros/src/opencv_demo/src/palm.xml")) {
			printf("Error loading cascade XML file\n");
		}
    }

    ~ImageConverter() {
        cv::namedWindow("Raw image");
        cv::namedWindow("Processed image");
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
        std::vector<cv::Rect> hands;
		cvtColor(processedImage.image, processedImage.image, CV_BGR2GRAY );
		equalizeHist(processedImage.image, processedImage.image);
		cascade.detectMultiScale(processedImage.image, hands, 1.07, 3, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
		for( int i = 0; i < hands.size(); i++ ) {
			cv::Point center(hands[i].x + hands[i].width*0.5, hands[i].y + hands[i].height*0.5);
			cv::ellipse(processedImage.image, center, cv::Size( hands[i].width*0.5, hands[i].height*0.5),
						0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);
			cv::Mat handROI = processedImage.image(hands[i]);
		}
		if (hands.size() > 0) {
			printf("(%d, %d)\n", hands[0].x, hands[0].y);
			fflush(stdout);
		}
        
        cv::imshow("Raw image", rawImagePtr->image);
        cv::imshow("Processed image", processedImage.image);
        cv::waitKey(3);
        image_pub.publish(processedImage.toImageMsg());
    }
    
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

	cv::CascadeClassifier cascade;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
