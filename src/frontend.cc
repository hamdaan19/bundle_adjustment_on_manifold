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

    SIFTFeature sift; 
    ORBFeature orb;

    // FLANN based feature matching: https://docs.opencv.org/4.x/d5/d6f/tutorial_feature_flann_matcher.html

    for (int i = 0; i < 2; i++) {
        // Reading images
        cv::Mat imgL0 = cv::imread(imagePathArrayLeft[i], cv::IMREAD_GRAYSCALE);
        cv::Mat imgR0 = cv::imread(imagePathArrayRight[i], cv::IMREAD_GRAYSCALE);
        cv::Mat imgL1 = cv::imread(imagePathArrayLeft[i+1], cv::IMREAD_GRAYSCALE);

        cv::Mat descriptorsL0, descriptorsR0, descriptorsL1;
        std::vector<cv::KeyPoint> keyPointsL0, keyPointsR0, keyPointsL1;

        std::vector<std::vector<cv::DMatch>> goodCommonMatches;
        std::vector<cv::DMatch> goodMatchesL0R0;

        // sift._detector->detectAndCompute(imgL0, cv::noArray(), keyPointsL0, descriptorsL0);
        // sift._detector->detectAndCompute(imgR0, cv::noArray(), keyPointsR0, descriptorsR0);
        // sift._detector->detectAndCompute(imgL1, cv::noArray(), keyPointsL1, descriptorsL1);

        // std::vector<std::vector<cv::DMatch>> goodCommonMatches;

        // sift.match(descriptorsL0, descriptorsR0, descriptorsL1, &goodCommonMatches); 

        // ORB Detector
        orb._detector->detect(imgL0, keyPointsL0); 
        orb._descriptorExtractor->compute(imgL0, keyPointsL0, descriptorsL0);
        orb._detector->detect(imgR0, keyPointsR0);
        orb._descriptorExtractor->compute(imgR0, keyPointsR0, descriptorsR0);
        orb._detector->detect(imgL1, keyPointsL1);
        orb._descriptorExtractor->compute(imgL1, keyPointsL1, descriptorsL1);

        orb.match(descriptorsL0, descriptorsR0, descriptorsL1, &goodCommonMatches, &goodMatchesL0R0);

        std::cout << "Number of good common matches is: " << goodCommonMatches.size() << "\n";

        // cv::Mat outimg1;
        // cv::drawKeypoints(imgL0, keyPointsL0, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // cv::imshow("ORB特征点", outimg1);

        // // Draw matches
        // cv::Mat img_matches;
        // cv::drawMatches(imgL0, keyPointsL0, imgR0, keyPointsR0, goodMatchesL0R0, img_matches);
        // cv::imshow("Matches L0 R0", img_matches);

        // cv::waitKey(0);
    }

    return 0; 
}