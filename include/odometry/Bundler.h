#ifndef BUNDLER_H
#define BUNDLER_H

#include <opencv2/core.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <map> 

#include "odometry/Feature.h"

typedef std::vector<std::map<int, int>> DataAssociation; 
typedef std::vector<DataAssociation> DataAssociationArray; 

class Bundler {
    bool _init = false;

    public: 
    Bundler() {}
    ~Bundler() {}

    ORBFeature _orb; 
    int _frame = 0; 
    DataAssociationArray dataAssociationArray; 
    DataAssociation dataAssociation; 

    // Matrix that stores all triangulated 3D Points
    cv::Mat _landmarks3d; 

    void initializeBundleStereo(
        cv::Mat* imgL0,
        cv::Mat* imgR0,
        cv::Mat* imgL1
    ) {
        // This function initializes the variables _keyPointsL1, _descriptorsL1, _goodCommonMatchesL0R0L1
        // for further iterations. This function must be called for the first iteration only. 
 
        cv::Mat descriptorsL0, descriptorsR0;
        std::vector<cv::KeyPoint> keyPointsL0, keyPointsR0;

        _orb._detector->detect(*imgL0, keyPointsL0);
        _orb._descriptorExtractor->compute(*imgL0, keyPointsL0, descriptorsL0);
        _orb._detector->detect(*imgR0, keyPointsR0);
        _orb._descriptorExtractor->compute(*imgR0, keyPointsR0, descriptorsR0);
        _orb._detector->detect(*imgL1, _keyPointsL1);
        _orb._descriptorExtractor->compute(*imgL1, _keyPointsL1, _descriptorsL1);

        _orb.match(descriptorsL0, descriptorsR0, _descriptorsL1, &_goodCommonMatchesL0R0L1, nullptr, &_L1MatchedKeyPointIndices);

        // Triangulate stereo matched points

        // Defining array of array for image points in the stereo pair
        std::vector<cv::Mat> points2d;

        // Defining matrices for storing all corresponding keypoint matches between the stereo images and initializing the matrices which it the first matching keypoints. 
        cv::Mat allPointsL0 = (cv::Mat_<double>(2, 1) << keyPointsL0[_goodCommonMatchesL0R0L1[0][0].queryIdx].pt.x, keyPointsL0[_goodCommonMatchesL0R0L1[0][0].queryIdx].pt.y);
        cv::Mat allPointsR0 = (cv::Mat_<double>(2, 1) << keyPointsR0[_goodCommonMatchesL0R0L1[0][0].trainIdx].pt.x, keyPointsR0[_goodCommonMatchesL0R0L1[0][0].trainIdx].pt.y); 
        
        // Data association map
        std::map<int, int> da; 
        // Adding the first association
        da[_goodCommonMatchesL0R0L1[0][1].trainIdx] = 0; 

        // Iterate through every good common match
        for (int i = 1; i < _goodCommonMatchesL0R0L1.size(); ++i) {
            // Extracting matching keypoints in the stereo pair L0, R0
            cv::Point2f leftStereoPoint = keyPointsL0[_goodCommonMatchesL0R0L1[i][0].queryIdx].pt;
            cv::Point2f rightStereoPoint = keyPointsR0[_goodCommonMatchesL0R0L1[i][0].trainIdx].pt;
            // Creating cv::Mat out of cv::Point2f
            cv::Mat matPointL0 = (cv::Mat_<double>(2, 1) << leftStereoPoint.x, leftStereoPoint.y);
            cv::Mat matPointR0 = (cv::Mat_<double>(2, 1) << rightStereoPoint.x, rightStereoPoint.y);

            cv::hconcat(allPointsL0, matPointL0, allPointsL0);
            cv::hconcat(allPointsR0, matPointR0, allPointsR0);

            // Creating correspondences between observations in L0 and triangulated 3D landmarks
            int keyPointIndexL1 = _goodCommonMatchesL0R0L1[i][1].trainIdx;
            da[keyPointIndexL1] = i; 
        }

        // Adding the std::map object to the vector
        dataAssociation.push_back(da); 
        
        // Storing the matrices with the corresponding matches in as std::vector
        points2d.push_back(allPointsL0); 
        points2d.push_back(allPointsR0); 

        // Reprojection Matrices
        std::vector<cv::Mat> projectionMatrices = {
            (cv::Mat_<double>(3, 4) << 
                7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00), // Projection matrix of L0 image
            (cv::Mat_<double>(3, 4) << 
                7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02, 
                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00)  // Projection matrix of R0 image                                                                                                                                                                                     // Projection matrix of R0 image
        };

        // Triangulate Points
        cv::sfm::triangulatePoints(points2d, projectionMatrices, _landmarks3d);

        std::vector<cv::Point3f> landmarks3d; 
        std::vector<cv::Point2f> observations2d; 

        // Perspective N Point for relative motion
        for (std::map<int,int>::iterator iter = da.begin(); iter != da.end(); ++iter) {
            cv::Point2f pt2d = _keyPointsL1[iter->first].pt;
            cv::Point3f pt3d = cv::Point3f(_landmarks3d.at<double>(0,iter->second), _landmarks3d.at<double>(1,iter->second), _landmarks3d.at<double>(2,iter->second));
            
            std::cout << "x: " << pt3d.x << " y: " << pt3d.y << " z: " << pt3d.z << "\n";
            
            observations2d.push_back(pt2d); 
            landmarks3d.push_back(pt3d); 
        }

        cv::Mat leftCameraMatrix = (cv::Mat_<double>(3,3) << 
        7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
        0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
        0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);

        cv::Mat rvec, tvec;
        cv::Mat distCoeffs = (cv::Mat_<double>(4,1) << 0, 0, 0, 0);

        bool success = cv::solvePnP(landmarks3d, observations2d, leftCameraMatrix, distCoeffs, rvec, tvec); 

    }

    void BundlerStereo (
        cv::Mat* imgR1,
        cv::Mat* imgL2
    ) {
        cv::Mat descriptorsR1, descriptorsL2;
        std::vector<cv::KeyPoint> keyPointsR1, keyPointsL2;

        std::vector<std::vector<cv::DMatch>> goodCommonMatchesL1R1L2;

        // Detecting key points and computing descriptors
        _orb._detector->detect(*imgR1, keyPointsR1);
        _orb._descriptorExtractor->compute(*imgR1, keyPointsR1, descriptorsR1);
        _orb._detector->detect(*imgL2, keyPointsL2);
        _orb._descriptorExtractor->compute(*imgL2, keyPointsL2, descriptorsL2);

        // Defining unordered sets
        std::vector<int> queryKeyPointIndices, train1KeyPointIndices;

        // Match features across three images
        _orb.match(_descriptorsL1, descriptorsR1, descriptorsL2, &goodCommonMatchesL1R1L2, &queryKeyPointIndices, &train1KeyPointIndices);

        // Find comming indices between queryMatchedKeyPointIndices and _L1MatchedKeyPointIndices
        std::vector<int> common = findCommon(&queryKeyPointIndices, &_L1MatchedKeyPointIndices);

        for (int n : common) {
            std::cout << "common: " << n << "\n"; 
        }

        std::cout << "Number of common indices: " << common.size() << "\n"; 

        // The Required Usual
        _frame += 1; 
    }

    std::vector<int> findCommon(std::vector<int>* array1, std::vector<int>* array2) {
        
        std::vector<int> common; 

        for (std::vector<int>::iterator i = array1->begin(); i != array1->end(); ++i)
        {
            if (std::find(array2->begin(), array2->end(), *i) != array2->end())
            {
                common.push_back(*i);
            }
        }

        return common; 
    }

    // void associateData(int cameraIdx, )

    // void findCommonAndAssociateData(std::vector<int> *L1Indices, std::vector<int> *prevL1Indices, std::vector<int>* L2Indices, std::vector<int>* common)
    // {
    //     for (int i = 0; i < L1Indices->size(); ++i)
    //     {
    //         int idx = L1Indices->at(i);
    //         if (std::find(prevL1Indices->begin(), prevL1Indices->end(), idx) != prevL1Indices->end())
    //         {
    //             common->push_back(idx);
                
    //         }
    //     }

    // }

    private: 
        std::vector<std::vector<cv::DMatch>> _goodCommonMatchesL0R0L1;
        std::vector<cv::KeyPoint> _keyPointsL1; 
        cv::Mat _descriptorsL1; 
        std::vector<int> _L1MatchedKeyPointIndices; 
};

#endif // BUNDLER_H