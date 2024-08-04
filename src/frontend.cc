#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "odometry/Utils.h"
#include "odometry/Feature.h"
#include "odometry/Bundler.h"

std::string DATASET_PATH; 
Utilities myUtils; 

int main(int argc, char** argv) {

    if (argc != 2) {
        std::cout << "[ERROR]: Invalid number of arguments. Correct usage: $./<your_executable> arg1\n";
        std::cout << "         arg1 := <path-to-dataset>\n";  
        return 1; 
    } else {
        DATASET_PATH = argv[1]; 
        std::cout << "[INFO]: Given dataset path: " << DATASET_PATH << "\n"; 
    }

    std::vector<std::string> imagePathArrayLeft, imagePathArrayRight;

    std::string datasetPathLeft = DATASET_PATH+"/image_0";
    std::string datasetPathRight = DATASET_PATH + "/image_1";

    myUtils.getStereoImagePath(234, datasetPathLeft, datasetPathRight, &imagePathArrayLeft, &imagePathArrayRight);

    // FLANN based feature matching: https://docs.opencv.org/4.x/d5/d6f/tutorial_feature_flann_matcher.html

    cv::Mat imgL0 = cv::imread(imagePathArrayLeft[0], cv::IMREAD_GRAYSCALE);
    cv::Mat imgR0 = cv::imread(imagePathArrayRight[0], cv::IMREAD_GRAYSCALE);
    cv::Mat imgL1 = cv::imread(imagePathArrayLeft[1], cv::IMREAD_GRAYSCALE);

    Bundler bundler;
    bundler.initializeBundleStereo(&imgL0, &imgR0, &imgL1);

    // for (int i = 1; i < 2; i++) {
    //     // Reading images
    //     cv::Mat imgL1 = cv::imread(imagePathArrayLeft[i], cv::IMREAD_GRAYSCALE);
    //     cv::Mat imgR1 = cv::imread(imagePathArrayRight[i], cv::IMREAD_GRAYSCALE);
    //     cv::Mat imgL2 = cv::imread(imagePathArrayLeft[i+1], cv::IMREAD_GRAYSCALE);

    //     bundler.BundlerStereo(&imgR1, &imgL2);
    // }

    return 0; 
}

// cv::Mat outimg1;
// cv::drawKeypoints(imgL0, keyPointsL0, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
// cv::imshow("ORB特征点", outimg1);

// // Draw matches
// cv::Mat img_matches;
// cv::drawMatches(imgL0, keyPointsL0, imgR0, keyPointsR0, goodMatchesL0R0, img_matches);
// cv::imshow("Matches L0 R0", img_matches);

// cv::waitKey(0);