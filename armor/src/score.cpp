/**
 * @file
 * @author
 * @brief
 *      score -> resetFunc -> calFunction -> setCircle
 *
 *      getScore -> getValue
 *
 * @note
 *
 */
#include "score.h"

using namespace cv::ml;
namespace ly
{
    void score::trainSVM(vector<vector<uchar>> samples, vector<int> labels) {
        int sampleNum = samples.size();
        cv::Mat sampleMat(sampleNum, 32*32, CV_32FC1);
        cv::Mat labelMat(sampleNum, 1, CV_32SC1);
        for(int label: labels){
            std::cout << label;
        }
        std::cout << std::endl;

        for(int i=0;i<sampleNum;i++){
            for(int j=0;j<32*32;j++){
                sampleMat.at<float>(i, j) = samples[i][j];
            }
            labelMat.at<uchar>(i, 0) = (uchar)labels[i];
        }
        std::cout << sampleMat << std::endl;
        std::cout << labelMat << std::endl;
        // create SVM
        svm = SVM::create();
        // set params
        svm->setC(0.1);
        svm->setType(SVM::C_SVC);
        svm->setKernel(SVM::LINEAR);
        svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 1000, 1e-6));
        // train
        cv::Ptr<TrainData> trainData = TrainData::create(sampleMat, ROW_SAMPLE, labelMat);
        svm->train(trainData);
        svm->save(svmModel);
    }

    void score::trainBoost(vector<vector<uchar>> samples, vector<int> labels){
        int sampleNum = samples.size();
        cv::Mat sampleMat(sampleNum, 32*32, CV_32FC1);
        cv::Mat labelMat(sampleNum, 1, CV_32SC1);
        for(int label: labels){
            std::cout << label;
        }
        std::cout << std::endl;

        for(int i=0;i<sampleNum;i++){
            for(int j=0;j<32*32;j++){
                sampleMat.at<float>(i, j) = samples[i][j];
            }
            labelMat.at<uchar>(i, 0) = (uchar)labels[i];
        }
        std::cout << sampleMat << std::endl;
        std::cout << labelMat << std::endl;
        // create
        boost = Boost::create();
        // set
        boost->setBoostType(Boost::DISCRETE);
        boost->setMaxDepth(10);
        boost->setWeightTrimRate(0);
        boost->setWeakCount(500);
        // train
        cv::Ptr<TrainData> trainData = TrainData::create(sampleMat, ROW_SAMPLE, labelMat);
        boost->train(trainData);
        boost->save("../boost.xml");
    }

    //circle
    double circle::getValue(double x){
        if(radius_ == -1)
        {//line
            return (center_ + x*vec).y;
        }
        else
        {//circle
            double temp = radius_*radius_ - (x-center_.x)*(x-center_.x);
            if(temp<0)
            {
                std::cerr<<"circle radio too small"<<std::endl;
                return  0;
            }
            double y = sqrt(temp)+center_.y;

            if(y<=1&&y>=0)
            {
                return y;
            }
            else
            {
                return center_.y-sqrt(temp);
            }
        }
    }
    void circle::setCircle(cv::Point2f pt1,cv::Point2f pt2,cv::Point2f pt3)
    {
        double a = pt1.x - pt2.x;
        double b = pt1.y - pt2.y;
        double c = pt1.x - pt3.x;
        double d = pt1.y - pt3.y;
        double e = ((pt1.x*pt1.x - pt2.x*pt2.x) + (pt1.y*pt1.y - pt2.y*pt2.y)) / 2.0;
        double f = ((pt1.x*pt1.x - pt3.x*pt3.x) + (pt1.y*pt1.y - pt3.y*pt3.y)) / 2.0;
        double det = b * c - a * d;
        if( fabs(det) < 1e-5)
        {//line
            radius_ = -1;
            center_ = pt1;
            vec= (pt2-pt1);
            vec = vec/vec.x;
            return ;
        }
        double x0 = -(d * e - b * f) / det;
        double y0 = -(a * f - c * e) / det;
        radius_ = hypot(pt1.x - x0, pt1.y - y0);
        center_.x = x0;
        center_.y = y0;
    }

    score::score(param config):
            setMiddle_(config.best),setMin_(config.min),setMax_(config.max),leftCeo_(config.left_ceo),rightCeo_(config.right_ceo)
    {
        resetFunc();
    }

    void score::resetFunc()
    {
        calFunction(left_,setMiddle_,setMin_,leftCeo_);
        calFunction(right_,setMiddle_,setMax_,rightCeo_);
    }
    double score::getScore(double x)
    {
        if(x>=setMiddle_&&x<=setMax_)
        {//right
            return right_.getValue(x-setMiddle_);
        }
        else if(x<setMiddle_&&x>=setMin_)
        {//left
            return left_.getValue(x-setMiddle_);
        }
        else
        {//deal region
            return -1;
        }
    }
    /*!
     * @brief 用来构造打分函数的公式,利用两个点和曲率构造圆弧
     * @param middle 最优值
     * @param aside 最值
     * @Ceo 圆弧曲率
     */
    void score::calFunction(circle& C,double middle, double aside, double Ceo)
    {
        double a = aside-middle;
        cv::Point2d centerPt(0,1);
        cv::Point2d asidePt(a,0);
        cv::Point2d middlePt = (centerPt+asidePt)/2;
        //cal min circle
        cv::Point2d minCirCenter(0,0.5-a*a/2);
        double minCirRadius = centerPt.y-minCirCenter.y;
        double edge = sqrt((a/2)*(a/2) + (a*a/2)*(a*a/2));
        double deltaL = minCirRadius - edge;
        cv::Point2d edgeVec = middlePt-minCirCenter;
        //cal set circle
        double setEdge = edge + Ceo*deltaL;
        cv::Point2d setEdgeVec = setEdge/edge*edgeVec;
        cv::Point2d circle3 = setEdgeVec + minCirCenter;
        C.setCircle(centerPt,asidePt,circle3);
    }
}


