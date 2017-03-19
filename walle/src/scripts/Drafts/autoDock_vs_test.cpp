#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

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



Mat filtering(Mat input)
{
    Mat image2;
//    Mat image3 = Mat::zeros( input.size(), input.type() );

    cv::Size ksize;
    ksize.height = 9;
    ksize.width = 9;

    double alpha = 2.0;
    int beta = 50;

    //apply gaussian blur to the image
    GaussianBlur(input, image2, ksize, 1, 0);

    //apply contrast filter to the image
    image2.convertTo(image2, -1, alpha, beta);

    //crop out the top 1/3 of the image, as the dock won't be there
    Mat croppedImage = image2(Rect(0,image2.rows/3,image2.cols,image2.rows*2/3));

//    for( int y = 0; y < input.rows; y++ )
//        { for( int x = 0; x < input.cols; x++ )
//            { for( int c = 0; c < 3; c++ )
//                {
//        image3.at<Vec3b>(y,x)[c] =
//            saturate_cast<uchar>( alpha*( image2.at<Vec3b>(y,x)[c] ) + beta );
//             }
//        }
//        }

    return croppedImage;
}


//global variables
bool docking_complete = false;
bool start_docking= false;

bool moveToDock();


int main (int argc, char** argv)
{

    ros::init(argc, argv, "autoDock_test");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Explore complete. Initiating auto-dock.");


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

    string images[7];
    Mat im;
    Mat im2;

    Mat im_with_keypoints;


    // Storage for blobs
    vector<KeyPoint> keypoints;

    // Set up detector with params
    SimpleBlobDetector detector(params);

    for (int f=0; f<6; f++)
    {
        stringstream ss;
        //dock images dock0.png to dock5.png must be in same directory
        ss << "dock" << f << ".png" << endl;

        string fullfileName;
        ss >> fullfileName;
        images[f] = fullfileName;
    }


    for(int i=0; i<6; i++)
    {

         //Read image
        im = imread(images[i], IMREAD_GRAYSCALE );

        im = filtering(im);
        //this function still seg faults

        // Detect blobs
        detector.detect( im, keypoints);


        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
        // the size of the circle corresponds to the size of blob


        drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        // Show blobs
        imshow("keypoints", im_with_keypoints );
        waitKey(0);
    }




    docking_complete = moveToDock();


}



bool moveToDock()
{
    actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac_dock("dock_drive_action", true);


    ROS_INFO("Waiting for the AutoDocking action server to come up");
    ac_dock.waitForServer();

	kobuki_msgs::AutoDockingGoal goal_dock;

	ac_dock.sendGoal(goal_dock);
	ac_dock.waitForResult(ros::Duration(60.0));

	if(ac_dock.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM("Docking complete");
        return true;
    }
    else
    {
        ROS_INFO_STREAM("Auto-docking failed");
        ac_dock.cancelGoal();
        return false;
    }
}
