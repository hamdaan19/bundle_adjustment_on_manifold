#ifndef BUNDLER_H
#define BUNDLER_H

#include <opencv2/core.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <opencv2/calib3d.hpp>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <opencv2/core/eigen.hpp>

#include <vector>
#include <map> 

#include "odometry/Feature.h"

typedef std::vector<std::map<int, int>> DataAssociation; 
typedef std::vector<DataAssociation> DataAssociationArray; 

class Bundler {
    bool _init = false;

    public: 
    Bundler() {
        // Reprojection Matrices
        _projectionMatrices = {
            (cv::Mat_<double>(3, 4) << 
                7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00), // Projection matrix of Left camera
            (cv::Mat_<double>(3, 4) << 
                7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02, 
                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00)  // Projection matrix of Right camera                                                                                                                                                                                     // Projection matrix of R0 image
        };

        // Intrinsic parameters of the left camera matrix
        _leftCameraIntrinsicMatrix = (cv::Mat_<double>(3,3) << 
        7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
        0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
        0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);

        cv::Mat _leftCameraDistCoeffs = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
    }
    ~Bundler() {}

    ORBFeature _orb; 
    int _frame = 0; 
    DataAssociationArray dataAssociationArray; 
    DataAssociation dataAssociation; 

    // Matrix that stores all triangulated 3D Points
    cv::Mat _landmarks3d; 

    // Array for storing camera extrinsics. 
    std::vector<Eigen::Affine3d> cameraArray; 

    // Projection Matrices for left and right camera
    std::vector<cv::Mat> _projectionMatrices; 
    cv::Mat _leftCameraIntrinsicMatrix;
    cv::Mat _leftCameraDistCoeffs;

    void initializeBundleStereo(
        cv::Mat* imgL0,
        cv::Mat* imgR0,
        cv::Mat* imgL1
    ) {
        // This function initializes the variables _keyPointsL1, _descriptorsL1, _goodCommonMatchesL0R0L1
        // for further iterations. This function must be called for the first iteration only. 
 
        // Defining matrices to store descriptors for L0 and R0 images
        cv::Mat descriptorsL0, descriptorsR0;
        // Defining arrays to store keypoints for L0 and R0 images
        std::vector<cv::KeyPoint> keyPointsL0, keyPointsR0;
        // Note that the descriptors and keypoints for L1 image are stored in 
        // variables which are defined as class members. 

        // Detecting keypoints and computing descriptors
        _orb._detector->detect(*imgL0, keyPointsL0);
        _orb._descriptorExtractor->compute(*imgL0, keyPointsL0, descriptorsL0);
        _orb._detector->detect(*imgR0, keyPointsR0);
        _orb._descriptorExtractor->compute(*imgR0, keyPointsR0, descriptorsR0);
        _orb._detector->detect(*imgL1, _keyPointsL1);
        _orb._descriptorExtractor->compute(*imgL1, _keyPointsL1, _descriptorsL1);

        // Finding good common matches between the three images
        _orb.match(descriptorsL0, descriptorsR0, _descriptorsL1, &_goodCommonMatchesL0R0L1, nullptr, &_L1MatchedKeyPointIndices);

        // TRIANGULATING POINTS FROM STEREO MATCHES

        // Defining array of array for image points in the stereo pair
        // Here, first element of type cv::Mat is a 2xN matrix containing coordinates of features from the first image.
        // The second element of type cv::Mat is also a 2xN matrix containing the coordinates of corresponding features in the second image. 
        std::vector<cv::Mat> points2d; 

        // Defining matrices for storing all corresponding keypoint matches between the stereo images and initializing the matrices which it the first matching keypoints. 
        cv::Mat allImagePointsL0 = (cv::Mat_<double>(2, 1) << keyPointsL0[_goodCommonMatchesL0R0L1[0][0].queryIdx].pt.x, keyPointsL0[_goodCommonMatchesL0R0L1[0][0].queryIdx].pt.y);
        cv::Mat allImagePointsR0 = (cv::Mat_<double>(2, 1) << keyPointsR0[_goodCommonMatchesL0R0L1[0][0].trainIdx].pt.x, keyPointsR0[_goodCommonMatchesL0R0L1[0][0].trainIdx].pt.y); 
        
        // Data association map. DA stands for Data Association. 
        // First item of this map object (KEY) refers to the index of the keypoint of L1 image. 
        // Second item of this map object (VALUE) corresponds to the index of the 3D landmark in the _landmarks3d array
        std::map<int, int> da; 
        // Adding the first association
        da[_goodCommonMatchesL0R0L1[0][1].trainIdx] = 0; 

        // Iterate through every good common match
        for (int i = 1; i < _goodCommonMatchesL0R0L1.size(); ++i) {
            // Extracting matching keypoints in the stereo pair L0, R0
            cv::Point2f leftStereoPoint = keyPointsL0[_goodCommonMatchesL0R0L1[i][0].queryIdx].pt;
            cv::Point2f rightStereoPoint = keyPointsR0[_goodCommonMatchesL0R0L1[i][0].trainIdx].pt;
            // Creating cv::Mat out of cv::Point2f
            cv::Mat imgPointL0 = (cv::Mat_<double>(2, 1) << leftStereoPoint.x, leftStereoPoint.y);
            cv::Mat imgPointR0 = (cv::Mat_<double>(2, 1) << rightStereoPoint.x, rightStereoPoint.y);

            // Concatenating columns
            cv::hconcat(allImagePointsL0, imgPointL0, allImagePointsL0);
            cv::hconcat(allImagePointsR0, imgPointR0, allImagePointsR0);

            // Creating correspondences between observations in L0 and triangulated 3D landmarks
            int keyPointIndexL1 = _goodCommonMatchesL0R0L1[i][1].trainIdx;
            da[keyPointIndexL1] = i; 
        }

        // Adding the std::map object to the vector
        // dataAssociation is an object of type std::vector<std::map<int, int>>. Each element of dataAssociation 
        // corresponds to map object which stores data association for all the good common features detected in a frame (L1, L2, L3, ... Ln)
        dataAssociation.push_back(da); 
        
        // Storing the matrices with the corresponding matches in as std::vector
        points2d.push_back(allImagePointsL0); 
        points2d.push_back(allImagePointsR0); 

        // Reprojection Matrices
        std::vector<cv::Mat> projectionMatrices = {
            (cv::Mat_<double>(3, 4) << 
                7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00), // Projection matrix of Left camera
            (cv::Mat_<double>(3, 4) << 
                7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02, 
                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00, 
                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00)  // Projection matrix of Right camera                                                                                                                                                                                     // Projection matrix of R0 image
        };

        // Triangulate Points
        cv::sfm::triangulatePoints(points2d, projectionMatrices, _landmarks3d);

        std::vector<cv::Point3f> points3d; 
        std::vector<cv::Point2f> observations2d; 

        // Perspective N Point for relative motion
        for (std::map<int,int>::iterator iter = da.begin(); iter != da.end(); ++iter) {
            cv::Point2f pt2d = _keyPointsL1[iter->first].pt;
            cv::Mat mat3d = _landmarks3d.col(iter->second);
            cv::Point3f pt3d = cv::Point3f(mat3d.at<double>(0), mat3d.at<double>(1), mat3d.at<double>(2));

            observations2d.push_back(pt2d);
            points3d.push_back(pt3d);
        }

        // Intrinsic parameters of the left camera matrix
        cv::Mat leftCameraMatrix = (cv::Mat_<double>(3,3) << 
        7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
        0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
        0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);

        cv::Mat rvec, tvec;
        cv::Mat distCoeffs = (cv::Mat_<double>(4,1) << 0, 0, 0, 0);

        // Finally compute the relative transformation between L0 and L1 using Perspective-N-Point algorithm
        bool success = cv::solvePnP(points3d, observations2d, leftCameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);

        // Convert rodrigues rotation to 3x3 rotation matrix 
        cv::Mat rot; // Matrix to store 3x3 rotation 
        cv::Rodrigues(rvec, rot); 
        // Convert cv::Mat to Eigen::Mat, rotation part
        Eigen::Matrix3d R; 
        cv::cv2eigen(rot, R);
        // Convert cv::Mat to Eigen::Mat, translation part
        Eigen::Vector3d t; 
        cv::cv2eigen(tvec, t); 

        // Create a Eigen::Affine3d object and initialize it with the computed rotation and translation
        Eigen::Affine3d cameraL1 = Eigen::Affine3d::Identity(); 
        cameraL1.linear() = R; 
        cameraL1.translation() = t;
        // Append the camera parameters into the cameraArray
        cameraArray.push_back(cameraL1); 


        // Verification 
        for (int i = 0; i < points3d.size(); i++) {
            cv::Point3d p3d = points3d[i];
            Eigen::Vector3d p(p3d.x, p3d.y, p3d.z); 
            Eigen::Matrix<double,3,3> K;
            K << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
                0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
                0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00; 
            // Projecting 3d point
            Eigen::Vector3d unscaled2d = K * (R * p + t);
            double u_proj = unscaled2d(0) / unscaled2d(2);
            double v_proj = unscaled2d(1) / unscaled2d(2);

            std::cout << "p       : " << tvec.t() << "\n";
            std::cout << "unscaled: " << unscaled2d.transpose() << "\n";

            std::cout << "u     : " << observations2d[i].x << " v     : " << observations2d[i].y << "\n";
            std::cout << "u_proj: " << u_proj << " v_proj: " << v_proj << "\n----------------\n"; 
        }


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

        // Create std::map<int,int> object for data association
        std::map<int,int> da; // DA for data association
        
        // Arrays for computing relative transformation via Perspective-N-Point
        std::vector<cv::Point3f> points3d;
        std::vector<cv::Point2f> observations2d;

        for (int i = 0; i < goodCommonMatchesL1R1L2.size(); ++i)
        {
            int queryIdxL1 = goodCommonMatchesL1R1L2[i][0].queryIdx; 
            int trainIdxR1 = goodCommonMatchesL1R1L2[i][0].trainIdx;
            int trainIdxL2 = goodCommonMatchesL1R1L2[i][1].trainIdx;

            if (std::find(_L1MatchedKeyPointIndices.begin(), _L1MatchedKeyPointIndices.end(), queryIdxL1) != _L1MatchedKeyPointIndices.end())
            {
                
                // Do data association
                int landmarkIdx = dataAssociation[dataAssociation.size()-1][queryIdxL1];
                da[trainIdxL2] = landmarkIdx;

                cv::Point2f pt2d = keyPointsL2[trainIdxL2].pt;
                observations2d.push_back(pt2d);

                cv::Mat mat3d = _landmarks3d.col(landmarkIdx);
                cv::Point3f pt3d = cv::Point3f(mat3d.at<double>(0), mat3d.at<double>(1), mat3d.at<double>(2));
                points3d.push_back(pt3d); 

            } else {
                // TRIANGULATE POINTS
                cv::Mat imgPointL1 = (cv::Mat_<double>(2, 1) << _keyPointsL1[queryIdxL1].pt.x, _keyPointsL1[queryIdxL1].pt.y);
                cv::Mat imgPointR1 = (cv::Mat_<double>(2, 1) << keyPointsR1[trainIdxR1].pt.x, keyPointsR1[trainIdxR1].pt.y);

                std::vector<cv::Mat> pts2d {imgPointL1, imgPointR1}; 

                cv::Mat mat3d, transformed_mat3d; 

                // Triangulate Points
                cv::sfm::triangulatePoints(pts2d, _projectionMatrices, mat3d);

                // The triangulated 3D point is in reference frame of camera L1. We need to transform it to world reference frame. 
                Eigen::Vector3d l; // l stands for landmark
                cv::cv2eigen(mat3d, l);
                Eigen::Affine3d cameraPose = cameraArray[cameraArray.size()-1].inverse(); 
                Eigen::Vector3d transformed_l = cameraPose * l;
                transformed_mat3d = (cv::Mat_<double>(3,1) << transformed_l(0), transformed_l(1), transformed_l(2)); 

                // Concatenating columns
                cv::hconcat(_landmarks3d, transformed_mat3d, _landmarks3d);

                // Do data association
                da[trainIdxL2] = _landmarks3d.cols - 1;

                // Add points into array for computing relative motion using Perspective-N-Point
                observations2d.push_back(keyPointsL2[trainIdxL2].pt);
                cv::Point3f pt3d = cv::Point3f(transformed_mat3d.at<double>(0), transformed_mat3d.at<double>(1), transformed_mat3d.at<double>(2));
                points3d.push_back(pt3d);
            }
        }

        // Finally compute the relative transformation between L1 and L2 using Perspective-N-Point algorithm
        cv::Mat rvec, tvec; 
        bool success = cv::solvePnP(points3d, observations2d, _leftCameraIntrinsicMatrix, _leftCameraDistCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);

        // Convert rodrigues rotation to 3x3 rotation matrix
        cv::Mat rot; // Matrix to store 3x3 rotation
        cv::Rodrigues(rvec, rot);
        // Convert cv::Mat to Eigen::Mat, rotation part
        Eigen::Matrix3d R;
        cv::cv2eigen(rot, R);
        // Convert cv::Mat to Eigen::Mat, translation part
        Eigen::Vector3d t;
        cv::cv2eigen(tvec, t);

        // Create a Eigen::Affine3d object and initialize it with the computed rotation and translation
        Eigen::Affine3d cameraL2 = Eigen::Affine3d::Identity();
        cameraL2.linear() = R;
        cameraL2.translation() = t;
        // Append the camera parameters into the cameraArray
        cameraArray.push_back(cameraL2);

        // Updating variables
        _keyPointsL1 = keyPointsL2;
        _descriptorsL1 = descriptorsL2; 
        _L1MatchedKeyPointIndices = train1KeyPointIndices; 
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