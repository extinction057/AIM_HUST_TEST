#ifndef __SCORE_
#define __SCORE_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "tools.h"

#define SMALL 1
#define LARGE 0
using namespace cv::ml;
using namespace std;

namespace ly
{
    class circle
    {
    public:
        circle(){}
        double getValue(double x);
        void setCircle(cv::Point2f pt1,cv::Point2f pt2,cv::Point2f pt3);

        cv::Point2d center_;
        cv::Point2d vec;
        double radius_;
    };
   class score
   {
   public:
       score()= default;
       ~score() = default;
       void trainSVM(vector<vector<uchar>> samples, vector<int> labels);
       void trainBoost(vector<vector<uchar>> samples, vector<int> labels);
       void setSampleNum(int num);
       void setLabelNum(int num);
       explicit score(param config);
//       score(double middle,double min,double max,double leftCeo = 0, double rightCeo = 0);
       void resetFunc();
       double getScore(double x);

       circle left_;
       circle right_;
       double setMin_{};
       double setMax_{};
       double setMiddle_{};
       double leftCeo_{};
       double rightCeo_{};

       std::string svmModel = "../svm_numbers.xml";
       std::string svmModel_small = "../svm_small1.xml";
       std::string svmModel_large = "../svm_large.xml";
       std::string boost_path = "../boost.xml";

       template <typename M>
       float predict(M model, cv::Mat sampleMat) {
//           cv::Mat sampleMat(1, samples.size(), CV_32FC1);
//           for(int i=0;i<samples.size();i++){
//               sampleMat.at<float>(i) = (float)samples[i];
//           }
           return model->predict(sampleMat);
       }

   private:
       void calFunction(circle& C,double middle,double aside,double Ceo);

       int sampleNum = 10;
       int labelNum = 10;

       cv::Ptr<SVM> svm;
       cv::Ptr<SVM> svm1;
       cv::Ptr<SVM> svm2;

       cv::Ptr<Boost> boost;
//       int image_width = 32;
//       int image_height = 32;
       bool isLoad = false;
   };
}

#endif //__SCORE_
