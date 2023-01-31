#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    // ToDo : Add your code here
    cv::Mat imgSmooth;     //or: imgGray.clone();
    cv::GaussianBlur(imgGray, imgSmooth, cv::Size(5, 5), 2.0);

    // create filter kernels using the cv::Mat datatype both for x and y
    // ToDo : Add your code here
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    float sobel_y[9] = {-1, -2, -1,
                         0,  0,  0, 
                        +1, +2, +1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

    // apply filter using the OpenCv function filter2D()
    // ToDo : Add your code here
    cv::Mat imgResult_x, imgResult_y;
    cv::filter2D(imgSmooth, imgResult_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::filter2D(imgSmooth, imgResult_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // compute magnitude image based on the equation presented in the lesson 
    // ToDo : Add your code here
    int cols = imgSmooth.cols, rows = imgSmooth.rows;
    cv::Mat magnitude = imgGray.clone();   // or cv::Mat magnitude(imgGray.rows, imgGray.cols, imgGray.type());

    // IMPORTANT: 
    // if instead of 'unsigned char' we use 'float' (since CV_32F); we won't get the correct results!
    for(int i = 0; i < rows; i++)
    {            
        for(int j = 0; j < cols; j++)
            magnitude.at<unsigned char>(i, j) = sqrt(pow(imgResult_x.at<unsigned char>(i, j), 2) + 
                                             pow(imgResult_y.at<unsigned char>(i, j), 2));
    }

    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}