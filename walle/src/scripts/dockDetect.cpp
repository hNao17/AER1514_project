//Created 2017-03-22
//Updated 2017-03-24
//Node that publishes image size & position information about the docking station
//Autodocking node can use published information to drive towards the dock using visual-servoing
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;

/** Global Variables **/
static const string OPENCV_WINDOW = "Image window";
//bool autoDock_active = false;
//bool dockSucceed = true;

//ros::Subscriber sub_AD_active;
//ros::Subscriber sub_dockSucceed_status;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pos_err_pub_,size_pub_,detect_flag_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
   //image_sub_ = it_.subscribe("/astra/image_raw", 1,
    //  &ImageConverter::imageCb, this);
        image_sub_ = it_.subscribe("/astra/image", 1,
      &ImageConverter::imageCb, this);
//    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
//          &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    pos_err_pub_ = nh_.advertise<std_msgs::Float32>("/dockDetect/x_position_err",1000);
    size_pub_ = nh_.advertise<std_msgs::Float32>("/dockDetect/size",1000);
    detect_flag_pub_ = nh_.advertise<std_msgs::Bool>("/dockDetect/detection_flag",1000);

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

    //Make 2 copies of the image
    //Mat image1 = cv_ptr->image;
    Mat image2;
    Mat croppedImage;

    // store width of image
    float image_width = cv_ptr->image.cols;

    //Apply a Gaussian blur
    GaussianBlur(cv_ptr->image, image2, ksize, 1, 0);
    //Apply contrast filter
    image2.convertTo(image2, -1, 2.0, 50);
    //Crop away top 1/3 of image
    croppedImage = image2(Rect(0,image2.rows/3,image2.cols,image2.rows*2/3));


    //Detect blobs
    detector.detect(croppedImage,keypoints);
    std_msgs::Float32 x_pos_error,blob_size;
    std_msgs::Bool detectFlag;

    //inititialization of detection params
    detectFlag.data = false;
    float prev_y_value = 0;

    // Selection of best Blob
    for(vector<KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++){
      ROS_INFO_STREAM("size of blob is: " << blobIterator->size);
      ROS_INFO_STREAM("point is at: " << blobIterator->pt.x << " " << blobIterator->pt.y);

      if (prev_y_value<blobIterator->pt.y)	// choose the blob at lowest height
      {
    	  x_pos_error.data = blobIterator->pt.x-image_width/2.0;
    	  blob_size.data = blobIterator->size;

      }
      prev_y_value = blobIterator->pt.y;
      detectFlag.data = true;
    }
    ROS_INFO_STREAM("====================");
    drawKeypoints(croppedImage, keypoints, cv_ptr->image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

    // Publish the blob details
    pos_err_pub_.publish(x_pos_error);
    size_pub_.publish(blob_size);
    detect_flag_pub_.publish(detectFlag);
  }
};

/** Function Declarations **/
void docking_callback(const std_msgs::Bool& msg_startDocking);
void dockSucceed_status_callback(const std_msgs::Bool& msg_startDetect);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dockDetect");
    //ros::NodeHandle nh4;

    //subscribe to dockON_status, dockSucceed_Status topics
    //sub_AD_active = nh4.subscribe("dockON_Status",1000,docking_callback);
   // sub_dockSucceed_status = nh4.subscribe("dockSucceed_Status", 1000, dockSucceed_status_callback);

    //node remains idle while the navigation node is active
  	//ros::Rate rate(10);
	/*
    while(!autoDock_active)
    {
        ROS_INFO_STREAM("Waiting for Autodocking node to start");
        ros::spinOnce();
    }
*/
    //autodocking node is now active
    ImageConverter ic;
    ros::spin();

//
//    //if Turtlebot has not docked, start looking for dockin station
//    while(!dockSucceed)
//    {
//        ROS_INFO_STREAM("Looking for docking station");
//        ros::spinOnce();
//    }
//
//    //robot is now docked at station; leave node in idle state
//    ROS_INFO_STREAM("Autodocking node is finished. Place dockDetect node in stand-by mode");
//
//    return 0;
}

/*
void docking_callback(const std_msgs::Bool& msg_startDocking)
{
    if(!msg_startDocking.data)
    {
        autoDock_active = false;
        ROS_INFO_STREAM("Auto-docking node is not active. Robot is not at home position.");
    }

    else
    {
        autoDock_active = true;
        ROS_INFO_STREAM("Auto-docking is active.");
    }
}

void dockSucceed_status_callback(const std_msgs::Bool& msg_startDetect)
{
    if(!msg_startDetect.data)
    {
        dockSucceed = false;
        ROS_INFO_STREAM("Auto-docking was not successful. Turn on visual servoing");
    }

    else
    {
        dockSucceed = true;
        ROS_INFO_STREAM("Auto-docking was sucessful.  Turn off visual servoing");
    }

}
*/

