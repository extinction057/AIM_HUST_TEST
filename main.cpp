#include <stdlib.h>
#include "main.h"

using namespace ly;
namespace fs = boost::filesystem;
using namespace std;


int main()
{
//    std::string path = "../Blue_1.avi";
//    cv::VideoWriter output(path, cv::VideoWriter::fourcc('M','J','P','G'), 50, cv::Size(1280, 1024));
//
//    config* config_ = new config();
//    cameraThread* camera_ = new cameraThread(config_->getConfigData().camDevice);
//    cv::waitKey(1000);
//    cv::Mat frame;
//    while(1){
//        frame = camera_->getFrame();
//        cv::imshow("frame", frame);
//        output.write(frame);
//        if(cv::waitKey(10) == 'q') break;
//    }
//    output.release();
//    return 0;

#if TRAIN_SVM
    score* score_ = new score();
    std::ifstream fileReader(samplePath);
    float* samples[rowNum];
    float labels[rowNum];
    for(int i=0;i<rowNum;i++){
        samples[i] = new float[7];
    }


    int index = 0;
    while(!fileReader.eof()){
        float feature[7], label;
        for(int i=0; i<7; i++){
            fileReader >> samples[index][i];
//            std::cout << feature[i] << " ";
        }
        fileReader >> labels[index];
//        std::cout << label << std::endl;
        index++;
    }
//    for(int i=0;i<rowNum; i++){
//        for(int j=0;j<7;j++){
//            std::cout << samples[i][j] << " ";
//        }
//        std::cout << labels[i] << std::endl;
//    }
//    std::cout << index << std::endl;
    score_->trainSVM(samples, labels, rowNum);
#elif TRAIN_BOOST
    score* score_ = new score();
    
    vector<vector<uchar>> samples;
    vector<int> labels;
    int id = 0;
    
    regex dir_regex("[0-9]+");
    
    fs::path root(picRoot);
    if(!fs::exists(root)){
        cout << "directory not exits." << endl;
        return -1;
    }
    fs::directory_iterator root_iter(root), end_iter;
    for(;root_iter != end_iter; root_iter++){
        if(fs::is_directory(root_iter->status()) && regex_match(root_iter->path().filename().string(), dir_regex)){
            fs::path picPath(root_iter->path().string());
            fs::directory_iterator picIter(picPath), endIter;
            string label = root_iter->path().filename().string();
            for(; picIter != endIter; picIter++){
                cv::Mat pic = cv::imread(picIter->path().string(), 0);
                cv::MatIterator_<uchar> matIter = pic.begin<uchar>(), end = pic.end<uchar>();
                vector<uchar> picVec;
                for(; matIter != end; matIter++){
//                    cout << (uchar)*matIter;
                    picVec.emplace_back((uchar)*matIter);
                }
                samples.emplace_back(picVec);
                labels.emplace_back(stoi(label));
                id++;
//                cout << endl;
            }
            std::cout << "label: " << stoi(label) << std::endl;
        }
    }

    score_->trainSVM(samples, labels);
//    score_->trainBoost(samples, labels);

//    string modelPath = "../boost.xml";
//    string testPic = "../data/pic/0/img_9.jpg";
//    cv::Mat testPicMat = cv::imread(testPic, 0);
//    vector<uchar> testPicVec;
//    cv::MatIterator_<uchar> begin = testPicMat.begin<uchar>(), end = testPicMat.end<uchar>();
//    for(; begin != end; begin++){
//        testPicVec.emplace_back((uchar)*begin);
//    }
//    cv::Ptr<cv::ml::Boost> boost;
//    boost = boost->load(modelPath);
////    cout << testPicVec.size() << endl;
//    float result = score_->predict<cv::Ptr<cv::ml::Boost>>(boost, testPicVec);
//    cout << result << endl;
#else
    armorDetector* armorDetector_ = new armorDetector();
    while (1);
#endif
}

void test(){
    return;
}

