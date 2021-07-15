#include <tools.h>
#include "armorThread.h"

namespace ly{
    armorThread::armorThread(armor_param config, armor_param large,cam_param cam_config,lightBarThread *lightBar,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,cameraThread *camera):
    lightBar_(lightBar),serialPortRead_(serialPortRead),serialPortWrite_(serialPortWrite),camera_(camera)
    {
        score_ = new score();
        pnp_ = new solvePNP(cam_config,serialPortRead_,serialPortWrite_);
        init(config, large);
        start(0);
//        counter_.countBegin();
    }

    armorThread::~armorThread()
    {
        delete lightBar_;
        delete score_;
        delete pnp_;
    }

    void armorThread::process()
    {
        std::priority_queue<lightBarNode> lightBar = lightBar_->getLightBarQue();       //获取检测到的灯条
        if(lightBar.size()>1)                                                           //有两个以上灯条
        {
            matchLightBar(lightBar);                                                    //从检测到的lightBar中检测armor
            if(armor_.empty())
            {
                //cout << "armor empty." << endl;
                return;
            }
            armorNode bestArmor = armor_.top();

//            std::cout << bestArmor.sum << std::endl;

//            std::cout <<
//                                           "lightBarRatio:    "  << bestArmor.lightBarRatio << std::endl <<
//                                           "lengthWidthRatio: " << bestArmor.lengthWidthRatio << std::endl <<
//                                           "angle:            " << bestArmor.angle << std::endl <<
//                                           "area:             " << bestArmor.area << std::endl <<
//                                           "parallelism:      " << bestArmor.parallelism << std::endl <<
//                                           "lastCenter:       " << bestArmor.deltaCenter << std::endl <<
//                                           "horizontalPara:   " << bestArmor.horizontalParallelism << std::endl <<
//                      std::endl;

//            std::cout <<
//                      "lightBarRatio:    " << bestArmor.LBR_ceo << std::endl <<
//                      "lengthWidthRatio: " << bestArmor.LWR_ceo << std::endl <<
//                      "angle:            " << bestArmor.angle_ceo << std::endl <<
//                      "area:             " << bestArmor.area_ceo << std::endl <<
//                      "parallelism:      " << bestArmor.parallelism_ceo << std::endl <<
//                      "lastCenter:       " << bestArmor.lastCenter_ceo << std::endl <<
//                      "horiPara:         " << bestArmor.hp_ceo << std::endl <<
//                      std::endl;
//
//            std::cout <<
//                      "large_lightBarRatio:    " << bestArmor.large_LBR_ceo << std::endl <<
//                      "large_lengthWidthRatio: " << bestArmor.large_LWR_ceo << std::endl <<
//                      "large_angle:            " << bestArmor.large_angle_ceo << std::endl <<
//                      "large_area:             " << bestArmor.large_area_ceo << std::endl <<
//                      "large_parallelism:      " << bestArmor.large_parallelism_ceo << std::endl <<
//                      "large_lastCenter:       " << bestArmor.large_lastCenter_ceo << std::endl <<
//                      "large_horiPara:         " << bestArmor.large_hp_ceo << std::endl <<
//                      std::endl;

            //std::cout << "sum: " << bestArmor.sum << " sumL: " << bestArmor.sum_large << std::endl;
            if(bestArmor.sum > 4.5 || bestArmor.sum_large > 4.5)
            {
#if TIMEIT
                clock_t startTime = clock();
#endif

#if VISUALIZE
                cv::Mat armorRoi = getArmorRoi(bestArmor);
                if(armorRoi.empty())
                {
                    return;
                }
                static int id = 0;
                if(id % 10 == 0){
                    mutex_pic_.lock();
                    cv::imwrite(path + std::to_string(id/10) + ".jpg", armorRoi);
                    mutex_pic_.unlock();
                }
                id++;
                std::cout << "id: " << id << std::endl;
#endif
#if PREDICT
                cv::Mat armorRoi = getArmorRoi(bestArmor);
                if(armorRoi.empty())
                {
                    return;
                }
                if(armorRoi.size().height != 32 || armorRoi.size().width != 32){
                    return;
                }

                cv::Mat armor;
                cv::cvtColor(armorRoi, armor, cv::COLOR_BGR2GRAY);

//                std::cout << armor << std::endl;
                armor.convertTo(armor, CV_32FC1);


//                std::cout << armor << std::endl;
//                cv::imshow("gray_armor", armor);
                cv::Mat armorMat = armor.reshape(0, 1);

//                std::cout << "size: " << armorMat.size() << " value: " << armorMat << std::endl;

                if(!isLoad){
                    model = model->load(score_->svmModel);
                    isLoad = true;
                }

                int result = (int)score_->predict<cv::Ptr<cv::ml::SVM>>(model, armorMat);

#if TIMEIT
                clock_t  endTime = clock();
                cout << "classifier time cost(ms): " << (endTime-startTime)*1.0/CLOCKS_PER_SEC*1000 << endl;
#endif
                std::cout << "result: " << result << std::endl;
                std::cout << (bestArmor.isLarge?"large":"small") << std::endl;
                switch (result) {
                    case 1:
                    case 8:
                        if(bestArmor.isLarge){
                            pnp_->solve(armor_.top(), LARGE);
                            update_ = true;
                        }
                        break;
                    case 2:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 7:
                        if(!bestArmor.isLarge){
                            pnp_->solve(armor_.top(), SMALL);
                            update_ = true;
                        }
                        break;
                    default:
                        break;
                }

#else
                pnp_->solve(armor_.top(), SMALL,armor_.top().receiveData_);
                update_ = true;
#endif
            }

//            else{
//                while(!armor_.empty()){
//                    armor_.pop();
//                }
//            }

//            shootType_ = serialPortRead_->getReceiveMsg();
//            shootType_.shootStatus = SMALL;
//            if(shootType_.shootStatus == SMALL){
//                if(bestArmor.sum > 4.5 && label1 == 1){
//
//                    pnp_->solve(bestArmor, SMALL);
//                    update_ = true;
//                }
//                else{
//                    while(!armor_.empty()){
//                        armor_.pop();
//                    }
//                }
//            }
//            // TODO 重新做数据集
//            else if(shootType_.shootStatus == LARGE){
//                if(label2 == 1 && bestArmor.area > 2000 && bestArmor.lengthWidthRatio > 3.5){
//                    pnp_->solve(bestArmor, LARGE);
//                    update_ = true;
//                }
//                else{
//                    while(!armor_.empty()){
//                        armor_.pop();
//                    }
//                }
//            }
//
//            if(bestArmor.lengthWidthRatio < 3.5 && label1 == 1)
//            {
//                pnp_->solve(bestArmor, SMALL);
////                lastStatus = SMALL;
//                update_ = true;
//            }
//            else if(label2 == 1){
//                pnp_->solve(bestArmor, LARGE);
////                lastStatus = LARGE;
//                update_ = true;
//            }
//            else{
//                while(!armor_.empty()){
//                    armor_.pop();
//                }
//            }
        }

        if(debug_ && !armor_.empty())
        {
            std::priority_queue<armorNode> armor_copy = armor_;             // 把armor_传递给lightBarThread，并可视化
            lightBar_->setArmorQue(armor_copy);
        }
        if(update_)
        {
            update_ = false;
            //std::cout<<"armor cost:"<<counter_.countEnd()<<std::endl;
        }
    }

    cv::Mat armorThread::getArmorRoi(armorNode bestArmor) {
        cv::Point2f lt1 = bestArmor.corner[0];
       cv::Point2f rt1 = bestArmor.corner[1];
        cv::Point2f rb1 = bestArmor.corner[2];
        cv::Point2f lb1 = bestArmor.corner[3];


        // 放大高度
        cv::Point2f lt = {constraint(lt1.x+(lb1.x-lt1.x)/2, (float)0., (float)1280.), constraint(lt1.y-(lb1.y-lt1.y)/2, (float)0., (float)1024.)};
        cv::Point2f rt = {constraint(rt1.x+(rb1.x-rt1.x)/2, (float)0., (float)1280.), constraint(rt1.y-(rb1.y-rt1.y)/2, (float)0., (float)1024.)};
        cv::Point2f lb = {constraint(lb1.x-(lb1.x-lt1.x)/2, (float)0., (float)1280.), constraint(lb1.y+(lb1.y-lt1.y)/2, (float)0., (float)1024.)};
        cv::Point2f rb = {constraint(rb1.x-(rb1.x-rt1.x)/2, (float)0., (float)1280.), constraint(rb1.y+(rb1.y-rt1.y)/2, (float)0., (float)1024.)};

        // 预裁减
        cv::Point2f minRectLt = {fmin(lt.x, lb.x), fmin(lt.y, rt.y)};
        cv::Point2f minRectRb = {fmax(rt.x, rb.x), fmax(lb.y, rb.y)};

        cv::Mat srcImage = lightBar_->pic_.mat;

        int x = (int)minRectLt.x;
        int y = (int)minRectLt.y;
        int width = (int)fmax(fmin((minRectRb.x-minRectLt.x), 1280 - x), 0);
        int height = (int)fmax(fmin((minRectRb.y-minRectLt.y), 1024 - y), 0);
        cv::Mat roiImage = srcImage(cv::Rect(x, y, width, height));
        cv::Mat dstImage = cv::Mat::zeros(srcImage.rows,srcImage.cols,srcImage.type());

        lt -= minRectLt;
        rt -= minRectLt;
        lb -= minRectLt;
        rb -= minRectLt;

        cv::Point2f lt_dst = lt;
        cv::Point2f rt_dst = {fmin(lt.x+roiSize.width, (float)width), lt.y};
        cv::Point2f lb_dst = {lt.x, fmin(lt.y+roiSize.height, (float)height)};

        cv::Point2f src[3] = {lt, rt, lb};
        cv::Point2f dst[3] = {lt_dst, rt_dst, lb_dst};

        cv::Mat warp_mat = cv::getAffineTransform(src, dst);
        //std::cout<<"7"<<std::endl;
        if(abs(warp_mat.at<int>(0,0))<1e-5&&abs(warp_mat.at<int>(0,1))<1e-5&&abs(warp_mat.at<int>(0,2))<1e-5)
        {
            return cv::Mat();

        }
        else if(abs(warp_mat.at<int>(1,0))<1e-5&&abs(warp_mat.at<int>(1,1))<1e-5&&abs(warp_mat.at<int>(1,2))<1e-5)
        {
            return cv::Mat();
        };
        cv::warpAffine(roiImage, dstImage, warp_mat, dstImage.size());
        int x_ = (int)lt.x;
        int y_ = (int)lt.y;
        int width_ = (int)fmin(roiSize.width, roiImage.size().width - x_);
        int height_ = (int)fmin(roiSize.height, roiImage.size().height - y_);
        return dstImage(cv::Rect(x_, y_, width_, height_));        return dstImage;
    }
}


