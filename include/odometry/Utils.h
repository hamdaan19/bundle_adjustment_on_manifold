#include <vector>
#include <string>

using namespace std; 

class Utilities {

    public:
        void getStereoImagePath (
            const int numImages,
            string imagesPathL,
            string imagesPathR,
            vector<string> *imagePathsR,
            vector<string> *imagePathL
        );
}; 