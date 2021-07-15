#include "buff.h"
#include<opencv2/opencv.hpp>
#include <ctime>


namespace ly {
    BUFF_Detector::BUFF_Detector(std::string cfg_Path,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,cam_param cam_config):
            serialPortRead_(serialPortRead),serialPortWrite_(serialPortWrite)
    {
        //1->red,0->blue
        int color_Type=0;//1->Blue,in RGB,it is Red    Vice versa
        new_Buff_Detector = new RM_BUFF_Dec(cfg_Path, color_Type,serialPortRead_,serialPortWrite_,cam_config);
        //new_Buff_Dectector
        communicator=new serialPort("/dev/ttyUSB0");
        ave_Fps = 0;
        c_Cnt = 0;
        is_Record = false;
        is_Find_mode= true;
        rec_Num = 0;
        wait_Num=0;
        find_Num=0;
        if (is_Record) {
            std::cout << "start recording\n";
            cap.open("../origin.avi", CV_FOURCC('M', 'P', '4', '2'), 60.0, cv::Size(1280, 1024));//960 540 / 1280 1024
        }
    }

    BUFF_Detector::~BUFF_Detector() {
        delete (new_Buff_Detector);
        delete (communicator);
        //delete(small_1);
        //delete(small_2);
    }
    bool BUFF_Detector::process(Mat src, cv::Point3f &est_Point) {
        int dec_Type =0;
        if (dec_Type == 0) {//input is RGB
            std::clock_t st, end;
            st = std::clock();
            bool isfind_Traj=false;

            if(is_Find_mode)
            {
                isfind_Traj = new_Buff_Detector->process(src);//after find the traj,wait 3 frame
                if (isfind_Traj && find_Num>3)
                    //if(find_Num>3)
                {

                    find_Num=0;
                    //std::cout<<"imu_pitch"<<(float)new_Data.pitch_Ang/100.f<<"imu_yaw"<<(float)new_Data.yaw_Ang/100.f<<std::endl;


//                    std::cout<<"***receiving message:pitch:"<<new_Data.pitch_Ang<<"  yaw:"<<new_Data.yaw_Ang<<"\n";
                    //std::cout<<"sending message: pitch:"<<pitch_Ang<<"  yaw:"<<yaw_Ang<<"\n";
//                    for(int nn=0;nn<1;nn++) {

//                    }
//                    if(new_Buff_Detector->ispredict == true)
//                    {
                    //new_Buff_Detector->cal_Ang(src.receiveData_);
//                        is_Find_mode=false;
//                        new_Buff_Detector->reset_All();
//                    }



                }
            }
            else{//wait until the robot is steady

                if(wait_Num==0)
                {
                    //communicator->genmsg_from_device(rec_Data);
                    wait_Num++;
                }
                else{
                    //while(!communicator->genmsg_from_device(new_Data))
                    //communicator->genmsg_from_device(new_Data);
                    //std::cout<<"receiving message:pitch:"<<new_Data.pitch_Ang<<"  yaw:"<<new_Data.yaw_Ang<<"\n";
                    if(src.receiveData_.yaw==rec_Data.yaw && src.receiveData_.pitch==rec_Data.pitch)
                    {
                        is_Find_mode=true;
                        wait_Num=0;
                    }
                    else
                        rec_Data.pitch = src.receiveData_.pitch;
                    rec_Data.yaw = src.receiveData_.yaw;
                }

                //std::cout<<"wait and start new one\n";
                is_Find_mode=true;
                wait_Num=0;
            }
            end = std::clock();
            double frame_Rate = (double) CLOCKS_PER_SEC / (end - st);
            cv::Mat show_Img;
            if(is_Find_mode) {
                if(isfind_Traj)
                    find_Num++;
                ave_Fps = ave_Fps + (double) (end - st);
                rec_Num++;
                std::string fps = std::to_string(frame_Rate);
                show_Img = new_Buff_Detector->show_Res(src.mat);
                old_Img=show_Img.clone();
                cv::putText(show_Img, fps, cv::Point(10, 30), 1, 1, cv::Scalar(0, 0, 255));
                cv::imshow("res", show_Img);
            }
            else{
                show_Img=old_Img.clone();
                std::string fps = std::to_string(frame_Rate);//may change
                cv::putText(show_Img, fps, cv::Point(10, 30), 1, 1, cv::Scalar(0, 0, 255));
//                cv::imshow("res", show_Img);
            }
            if (is_Record) {
                cap << show_Img;
            }
            char c = cv::waitKey(10);
            if (c == 'q') {
                if (cap.isOpened()) {
                    cap.release();
                    is_Record = false;
                    std::cout << "end recording\n";
                } else
                    std::cout << "already stop recording\n";
            }
            //std::cout << "ave_Fps:" << 1.0 / (ave_Fps / (rec_Num * CLOCKS_PER_SEC)) << "\n";
        } else {
//            std::clock_t st, end;
//            st = std::clock();
//            buff_Detector->process(src);
//            end = std::clock();
//            ave_Fps = ave_Fps + (double) (end - st);
//            rec_Num++;
//            double frame_Rate = (double) CLOCKS_PER_SEC / (end - st);
//            std::string fps = std::to_string(frame_Rate);
//            cv::Mat show_Img = buff_Detector->show_Res(src);
//            cv::putText(show_Img, fps, cv::Point(10, 30), 1, 1, cv::Scalar(0, 0, 255));
//            cv::imshow("res", show_Img);
//            if (is_Record) {
//                cap << show_Img;
//            }
//            char c = cv::waitKey(1);
//            if (c == 'q') {
//                if (cap.isOpened()) {
//                    cap.release();
//                    is_Record = false;
//                    std::cout << "end recording\n";
//                } else
//                    std::cout << "already stop recording\n";
//            }
//            std::cout << "ave_Fps:" << 1.0 / (ave_Fps / (rec_Num * CLOCKS_PER_SEC)) << "\n";
        }
        /*
        if (true) {
            cv::imshow("res", src);
            if (is_Record) {
                cap << src;
            }
            char c = cv::waitKey(1);
            if (c == 'q') {
                if (cap.isOpened()) {
                    cap.release();
                    is_Record = false;
                    std::cout << "end recording\n";
                } else {
                    is_Record = true;
                    cap.open("../" + std::to_string(rec_Num) + ".avi", CV_FOURCC('M', 'P', '4', '2'), 60.0,
                             cv::Size(1280, 1024));
                    std::cout << "start recording,../" << std::to_string(rec_Num) + ".avi\n";
                    rec_Num++;
                }
            }
        }
         */
    }
}
//            if (nxtEdge < preEdge && preEdge == edgeSet[2])
//            {
//                quadrant = 0;
//                targets[0].angle = 180+atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 3) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 3) % 4].y)*180.f/3.1415926;
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);
//
//            }
//            else if (nxtEdge > preEdge && nxtEdge == edgeSet[2])
//            {
//                quadrant = 1;
//                targets[0].angle = atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 1) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 1) % 4].y)*180.f/3.1415926;;
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
//
//            }
//            else if (nxtEdge < preEdge && preEdge == edgeSet[3])
//            {
//                quadrant = 2;
//                targets[0].angle = atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 3) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 3) % 4].y)*180.f/3.1415926;
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);
//            }
//            else if (nxtEdge > preEdge && nxtEdge == edgeSet[3])
//            {
//                quadrant = 3;
//                targets[0].angle = -180+atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 1) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 1) % 4].y)*180.f/3.1415926;
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);
//                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
//            }
