//#include <stdio.h>
//#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

static const string OPENCV_WINDOW = "Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    Size ksize;
    ksize.height = 9;
    ksize.width = 9;

    double alpha = 2.0;
    int beta = 50;
  
    // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 0;
	params.maxThreshold = 175;
    
    //filter by color
    params.filterByColor = true;
    params.blobColor = 0;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 500;
    params.maxArea = 10000;

	// Filter by Circularity
	params.filterByCircularity = false;
	//params.minCircularity = 0.785;
    params.maxCircularity = 0.999;

	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.1;

	// Filter by Inertia
	params.filterByInertia = true;
    params.minInertiaRatio = 0.15;
	//params.maxInertiaRatio = 0.5;

    Mat im_with_keypoints;
    
    // Storage for blobs
    vector<KeyPoint> keypoints;

    // Set up detector with params
    SimpleBlobDetector detector(params);

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    //Make 2 copies of the image
    Mat image1 = cv_ptr->image;
    Mat image2;
    Mat croppedImage;
    
    //Apply a Gaussian blur
    GaussianBlur(image1, image2, ksize, 1, 0);  
    //Apply contrast filter
    image2.convertTo(image2, -1, 2.0, 50);
    //Crop away top 1/3 of image
    croppedImage = image2(Rect(0,image2.rows/3,image2.cols,image2.rows*2/3));
    
    //Re-assign the cropped image
    //cv_ptr->image = croppedImage;
    
    //Detect blobs
    detector.detect(croppedImage,keypoints);
    
    drawKeypoints(croppedImage, keypoints, cv_ptr->image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


    // Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

