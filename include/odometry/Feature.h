#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class SIFTFeature {

    public:

        cv::Ptr<cv::SIFT> _detector;
        cv::Ptr<cv::DescriptorMatcher> _matcher;

        // matcher parameters
        double _matchRatioThreshold = 0.7;
        cv::DescriptorMatcher::MatcherType _matcherType = cv::DescriptorMatcher::FLANNBASED;

        SIFTFeature(int numFeatures = 0, int numOctaveLayers = 5, double contrastThreshold = 0.03, double edgeThreshold = 10, double sigma = 1) :
        _nfeatures(numFeatures),
        _nOctaveLayers(numOctaveLayers),
        _contrastThreshold(contrastThreshold), 
        _edgeThreshold(edgeThreshold),
        _sigma(sigma)
        {
            _detector = cv::SIFT::create(_nfeatures, _nOctaveLayers, _contrastThreshold, _edgeThreshold, _sigma);
            _matcher = cv::DescriptorMatcher::create(_matcherType);
        }

        void match(
            cv::Mat queryDescriptors, 
            cv::Mat trainDescriptor0, 
            cv::Mat trainDescriptor1, 
            std::vector<std::vector<cv::DMatch>>* goodCommonMatches
        ); 

    private:
        int _nfeatures;
        int _nOctaveLayers;
        double _contrastThreshold; 
        double _edgeThreshold; 
        double _sigma;

}; 

class ORBFeature {
    public:
        cv::Ptr<cv::ORB> _detector;
        cv::Ptr<cv::DescriptorMatcher> _matcher;
        cv::Ptr<cv::DescriptorExtractor> _descriptorExtractor; 

        cv::DescriptorMatcher::MatcherType _matcherType = cv::DescriptorMatcher::BRUTEFORCE_HAMMING; 

        ORBFeature(const int numFeatures = 20000):
        _nfeatures(numFeatures) 
        {
            _detector = cv::ORB::create(_nfeatures); 
            _matcher = cv::DescriptorMatcher::create(_matcherType); 
            _descriptorExtractor = cv::ORB::create(_nfeatures); 
        }

        void match(
            cv::Mat queryDescriptors,
            cv::Mat trainDescriptor0,
            cv::Mat trainDescriptor1,
            std::vector<std::vector<cv::DMatch>> *goodCommonMatches,
            std::vector<cv::DMatch> *goodMatches0 = nullptr, // good matches between queryDescriptors and trainDescriptor0
            std::vector<cv::DMatch> *goodMatches1 = nullptr  // good matches between queryDescriptors and trainDescriptor1
        );

    private:
        int _nfeatures; 
};

class FeatureMatcher {

}; 