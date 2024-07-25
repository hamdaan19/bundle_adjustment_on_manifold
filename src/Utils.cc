#include "odometry/Utils.h"

using namespace std;

void Utilities::getStereoImagePath(const int numImages, string datasetPathLeft, string datasetPathRight, vector<string> *imagePathArrayLeft, vector<string> *imagePathArrayRight)
{
    for (int img = 0; img < numImages; img++)
    {
        string str = std::to_string(img);
        size_t n = 6;

        str = std::string(n - str.size(), '0').append(str);

        string filenameL = datasetPathLeft + "/" + str + ".png";
        string filenameR = datasetPathRight + "/" + str + ".png";

        imagePathArrayLeft->push_back(filenameL);
        imagePathArrayRight->push_back(filenameR);
    }
}