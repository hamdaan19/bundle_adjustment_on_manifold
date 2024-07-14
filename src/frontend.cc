#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

std::string DATASET_PATH; 

int main(int argc, char** argv) {

    if (argc != 2) {
        std::cout << "[ERROR]: Invalid number of arguments. Correct usage: $./<your_executable> arg1\n";
        std::cout << "         arg1 := <path-to-dataset>\n";  
        return 1; 
    } else {
        DATASET_PATH = argv[1]; 
    }

    std::string img_path_1 = DATASET_PATH+"/image_0/000080.png";
    std::string img_path_2 = DATASET_PATH+"/image_1/000080.png";
    std::string img_path_3 = DATASET_PATH+"/image_0/000081.png";
    std::string img_path_4 = DATASET_PATH+"/image_1/000081.png";

    cv::Mat img1 = cv::imread(img_path_1, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(img_path_2, cv::IMREAD_GRAYSCALE);
    cv::Mat img3 = cv::imread(img_path_3, cv::IMREAD_GRAYSCALE);
    cv::Mat img4 = cv::imread(img_path_4, cv::IMREAD_GRAYSCALE);

    return 0; 
}