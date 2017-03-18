//#include <stdio.h>
//#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

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

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    cv_ptr->image.convertTo(cv_ptr->image, -1, 2.0, 50);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
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

//using namespace cv;
//
//int main()
//{
//    //if ( argc != 2 )
//    //{
//      //  printf("usage: DisplayImage.out <Image_Path>\n");
//        //return -1;
//    //}
//
//    Mat image;
//    image = imread( "dock0.png", IMREAD_GRAYSCALE );
//    //waitKey(5000);
//    Mat image2; 
//    Mat image3 = Mat::zeros( image.size(), image.type() );
//    
//    double alpha = 2.0;
//    int beta = 50;
//    
//    cv::Size ksize;
//    ksize.height = 9;
//    ksize.width = 9;
//
//    if ( !image.data )
//    {
//        printf("No image data \n");
//        return -1;
//    }
//    namedWindow("Display Image", WINDOW_AUTOSIZE );
//    
//    GaussianBlur(image, image2, ksize, 1, 0);
//    
//     for( int y = 0; y < image.rows; y++ )
//        { for( int x = 0; x < image.cols; x++ )
//            { for( int c = 0; c < 3; c++ )
//                {
//        image3.at<Vec3b>(y,x)[c] =
//            saturate_cast<uchar>( alpha*( image2.at<Vec3b>(y,x)[c] ) + beta );
//             }
//        }
//        }
//    
//    imshow("Display Image", image);
//    waitKey(0);
//    imshow("Filtered Image", image2);
//    waitKey(0);
//    imshow("Contrasted Image", image3);
//    waitKey(0);
//
//    return 0;
//}