#include "odometry/Feature.h"

void SIFTFeature::match(
    cv::Mat queryDescriptors,
    cv::Mat trainDescriptor0,
    cv::Mat trainDescriptor1,
    std::vector<std::vector<cv::DMatch>> *goodCommonMatches
) {
    // Creating array of arrays to store all matches
    std::vector<std::vector<cv::DMatch>> matches0, matches1; 
    // Matching query descriptors with first set of train descriptors and storing '2' best matches
    _matcher->knnMatch(queryDescriptors, trainDescriptor0, matches0, 2);
    // Matching query descriptors with second set of train descriptors and storing '2' best matches
    _matcher->knnMatch(queryDescriptors, trainDescriptor1, matches1, 2);

    for (int i = 0; i < matches0.size(); i++) // Iterate through every single descriptor-match
    {
        // Check if match for both matches for trainDescriptor0 and trainDescriptor1 are good
        // using Lowe's ratio test. If the matches for both train images are good, then store
        // them as good-common matches. 
        if (
            (matches0[i][0].distance < _matchRatioThreshold * matches0[i][1].distance) && 
            (matches1[i][0].distance < _matchRatioThreshold * matches1[i][1].distance)
        ) {
            // Order of storing: index of the match is give precedence
            std::vector<cv::DMatch> matchesArray{matches0[i][0], matches1[i][0]};
            goodCommonMatches->push_back(matchesArray);
        }
    }
}

void ORBFeature::match(
    cv::Mat queryDescriptors,
    cv::Mat trainDescriptor0,
    cv::Mat trainDescriptor1,
    std::vector<std::vector<cv::DMatch>> *goodCommonMatches,
    std::vector<cv::DMatch> *goodMatches0,
    std::vector<cv::DMatch> *goodMatches1
) {
    // Creating arrays to store matches
    std::vector<cv::DMatch> matches0, matches1;
    // Match query and first train descriptors
    _matcher->match(queryDescriptors, trainDescriptor0, matches0);
    // Match query and second train descriptors
    _matcher->match(queryDescriptors, trainDescriptor1, matches1);

    double minDistForMatches0 = matches0[0].distance;   // Initializing with the first value
    double minDistForMatches1 = matches1[0].distance;   // Initializing with the first value

    // Iterating through all matches for every query descriptors to find the minimum distances for both sets of train descriptors
    // This is useful for filtering the good matches. 
    for (int i = 0; i < queryDescriptors.rows; i++) {
        double dist0 = matches0[i].distance;
        double dist1 = matches1[i].distance;
        if ( dist0 < minDistForMatches0 ) minDistForMatches0 = dist0;
        if ( dist1 < minDistForMatches1 ) minDistForMatches1 = dist1;
    }

    // Filtering through matches to obtain good common matches
    for (int i = 0; i < queryDescriptors.rows; i++) {
        // When the distance between descriptors is greater than twice the minimum distance, 
        // the matching is considered incorrect. But sometimes the minimum distance will be 
        // very small, and an empirical value of 30 is set as the lower limit.

        if (
            ( matches0[i].distance < std::max(2*minDistForMatches0, 30.0) ) &&
            ( matches1[i].distance < std::max(2*minDistForMatches1, 30.0) )
        ) {
            // Order of storing: index of the match is give precedence
            std::vector<cv::DMatch> matchesArray{matches0[i], matches1[i]};
            goodCommonMatches->push_back(matchesArray);

            if (goodMatches0 != nullptr) goodMatches0->push_back(matches0[i]); 
            if (goodMatches1 != nullptr) goodMatches1->push_back(matches1[i]); 
        }
    }
}