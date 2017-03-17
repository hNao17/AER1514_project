/**
 * OpenCV SimpleBlobDetector Example
 *
 * Copyright 2015 by Satya Mallick <spmallick@gmail.com>
 * modified by Kasper and Venu
 *
 * requires images dock0.png to dock5.png in the same directoy
 */

#include "opencv2/opencv.hpp"
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

Mat filtering(Mat input)
{
    Mat image2; 
//    Mat image3 = Mat::zeros( input.size(), input.type() );
    
    cv::Size ksize;
    ksize.height = 9;
    ksize.width = 9;

    double alpha = 2.0;
    int beta = 50;
    
    GaussianBlur(input, image2, ksize, 1, 0);
    
    image2.convertTo(image2, -1, alpha, beta);
    
//    for( int y = 0; y < input.rows; y++ )
//        { for( int x = 0; x < input.cols; x++ )
//            { for( int c = 0; c < 3; c++ )
//                {
//        image3.at<Vec3b>(y,x)[c] =
//            saturate_cast<uchar>( alpha*( image2.at<Vec3b>(y,x)[c] ) + beta );
//             }
//        }
//        }
    
    return image2;
}

int main( int argc, char** argv )
{

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

}

