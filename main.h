#ifndef ARMORDETECTOR_MAIN_H
#define ARMORDETECTOR_MAIN_H

#include "armorDetector.h"
#include <regex>

#define TRAIN_SVM 0
#define TRAIN_BOOST 0

using namespace std;
namespace fs = boost::filesystem;
namespace ly{

    string samplePath = "../data/armor_feature_large.txt";
    string picRoot = "../data/pic";


    int rowNum = 3083;
}

#endif //ARMORDETECTOR_MAIN_H
