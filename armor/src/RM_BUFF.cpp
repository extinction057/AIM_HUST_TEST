//for current BUFF device
#include "RM_BUFF.h"
#include "transform.h"
#include "omp.h"
namespace ly {
    void m_ResizeByNN(uchar *input, uchar *output, int height_in, int width_in, int channels, int height_out, int width_out);

//todo:calculate the angle without pnp,but arctan
//todo:add communication module,both read and write
//todo:adjust logical process
//todo:turn it to pipeline


    RM_BUFF_Dec::RM_BUFF_Dec(std::string config_Path,int color_Type,serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite,cam_param cam_config):
            serialPortRead_(serialPortRead),serialPortWrite_(serialPortWrite)
    //0->Red 1->Blue
    {
        isfind_Armor = false;
        isfind_Traj = false;
        m_Color = color_Type;
        std::cout << "config path:" << "/home/angrypsyduck/AIM_HUST_DIR/AIM_HUST/config/new_Buff_cfg.yml" << "\n";
        cv::FileStorage cfg_Load("/home/angrypsyduck/AIM_HUST_DIR/AIM_HUST/config/new_Buff_cfg.yml", cv::FileStorage::READ);
        cfg_Load["color_thresh_red"] >> color_Thresh_red;
        cfg_Load["color_thresh_blue"] >> color_Thresh_blue;
        cfg_Load["element_size"] >> ele_Size;
        cfg_Load["armor_pts_min"] >> armor_Pts_min;
        cfg_Load["armor_pts_max"] >> armor_Pts_max;
        cfg_Load["far_armor_min"] >> far_Armor_min;
        cfg_Load["far_armor_max"] >> far_Armor_max;
        cfg_Load["u0"] >> cam_U0;
        cfg_Load["v0"] >> cam_V0;
        cfg_Load["f"] >> cam_F;
        cfg_Load["useroi"] >> isuse_Roi;
        isget_Roi = false;
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ele_Size, ele_Size));
        ele4blue = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        ele4red = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        std::cout << "BUFF parameter:\n";
        std::cout << "color_Type:" << color_Type << "  color_Thresh_red:" << color_Thresh_red << "  color_Thresh_blue:"
                  << color_Thresh_blue << "\n";
        std::cout << "element_Size:" << ele_Size << "\n";
        std::cout << "armor_pts_min:" << armor_Pts_min << "  armor_pts_max:" << armor_Pts_max << "\n";
        std::cout << "far_armor_min:" << far_Armor_min << "  far_armor_max:" << far_Armor_max << "\n";
        std::cout << "u0:" << cam_U0 << "  v0:" << cam_V0 << "  F:" << cam_F << "\n";
        param_x0 = 0;
        param_y0 = 0;
        param_r = 0;
        cur_Img = cv::Mat(cv::Size(640, 512), CV_8UC1);
        pnp_ = new solvePNP(cam_config,serialPortRead,serialPortWrite);
        transform_ = new transform(serialPortRead_,serialPortWrite_);
        isfirsttfA = false;
        isinitialize = false;
        isfitspd = false;

        //cv::waitKey(0);

    }

    void m_ResizeByNN(uchar *input, uchar *output, int height_in, int width_in, int channels, int height_out,
                      int width_out) {

        uchar *data_source = input;
        uchar *data_half = output;

        int bpl_source = width_in * 3;
        int bpl_dst = width_out * 3;

        int pos = 0;
        int sep = 0;
        uchar *sr = nullptr;
        uchar *hr = nullptr;
        float step = 0.0;
        float step_x = float(width_in) / float(width_out);
        float step_y = float(height_in) / float(height_out);

        for (int i = 0; i < height_out; i++) {
            for (int j = 0; j < width_out; j++) {
                sep = int(step_y * i);
                step = int(j * step_x);
                sr = data_source + sep * bpl_source;
                hr = data_half + i * bpl_dst + j * channels;
                pos = step * channels;
                memcpy(hr, sr + pos, channels);
            }
        }
        return;
    }





    bool RM_BUFF_Dec::fit_Traj_3D()
    {
        std::vector<float> circle;

        int num = long_Set_3D.size();
        int dim = 3;

        Eigen::MatrixXd M(num, dim);

        for (int i = 0; i < num; i++)
        {
            for (int j = 0; j < dim; j++)
            {
                M(i, j) = long_Set_3D[i][j];
            }
        }

        Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(num, 1);

        //式（6）
        Eigen::MatrixXd A = (M.transpose()*M).inverse()*M.transpose()*L1;

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num - 1, 3);

        for (int i = 0; i < num - 1; i++)
        {
            B.row(i) = M.row(i + 1) - M.row(i);
        }

        Eigen::MatrixXd L2 = Eigen::MatrixXd::Zero(num - 1, 1);
        for (int i = 0; i < num - 1; i++)
        {
            L2(i) = (M(i + 1, 0)*M(i + 1, 0) + M(i + 1, 1)*M(i + 1, 1) + M(i + 1, 2)*M(i + 1, 2)
                     - (M(i, 0)*M(i, 0) + M(i, 1)*M(i, 1) + M(i, 2)*M(i, 2))) / 2.0;
        }

        Eigen::MatrixXd D;
        //！！！矩阵合并前需要将合并后的矩阵 resize
        D.resize(4, 3);
        D << B.transpose()*B,
                A.transpose();

        Eigen::MatrixXd L3;
        Eigen::MatrixXd One31 = Eigen::MatrixXd::Ones(3, 1);
        L3.resize(4, 1);
        L3 << B.transpose()*L2,1;

        //式（7）
        Eigen::MatrixXd C = (D.transpose()*D).inverse()*D.transpose() * L3;

        //式（8）
        double radius = 0;
        for (int i = 0; i < num; i++)
        {
            Eigen::MatrixXd tmp = M.row(i) - C.transpose();
            radius = radius + sqrt(tmp(0)*tmp(0) + tmp(1)*tmp(1) + tmp(2)*tmp(2));
        }
        radius = radius / num;

        circle_3D_.Cx=C(0);
        circle_3D_.Cy=C(1);
        circle_3D_.Cz=C(2);
        circle_3D_.a=A(0);
        circle_3D_.b=A(1);
        circle_3D_.c=A(2);
        circle_3D_.radius=radius;
        std::cout<<"radius"<<radius<<"circle_3D_.Cx"<<circle_3D_.Cx<<std::endl;
        return false;
    }





    bool RM_BUFF_Dec::process(Mat src)
    {

        //cv::imshow("buff",buff_show);
        //cv::imshow("source",src);
        isfind_Armor = false;
        if(src.mat.empty())
        {
            return false;
        }
        real_Src = src.mat.clone();
        src_temp = src.mat.clone();
        receiveData_now = src.receiveData_;
        src_Img = cv::Mat(0.5 * src.mat.rows, 0.5 * src.mat.cols, src.mat.type());//0.5
        m_ResizeByNN(src.mat.data, src_Img.data, src.mat.rows, src.mat.cols, src.mat.channels(), src_Img.rows, src_Img.cols);
        init_Img();
        if (find_Armor())
        {
            return true;
        }
        return false;
    }

    void RM_BUFF_Dec::sortpoint(std::vector<cv::Point2f> Point)
    {
        float min_y = Point[0].y;
        int min_y_ = 0;
        for(int i =1;i<4;i++)
        {
            if(Point[i].y<min_y)
            {
                min_y = Point[i].y;
                min_y_ = i;
            }
        }
        Points2D.emplace_back(Point[min_y_]);

        double lenth1 = sqrt(pow(Point[min_y_].y - Point[(min_y_+1)%4].y,2)+pow(Point[min_y_].x - Point[(min_y_+1)%4].x,2));
        double lenth2 = sqrt(pow(Point[min_y_].y - Point[(min_y_+4-1)%4].y,2)+pow(Point[min_y_].x - Point[(min_y_+4-1)%4].x,2));
        if(lenth1<lenth2)
        {
            Points2D.emplace_back(Point[(min_y_+1)%4]);
            Points2D.emplace_back(Point[(min_y_+2)%4]);
            Points2D.emplace_back(Point[(min_y_+3)%4]);
        }
        else
        {
            Points2D.emplace_back(Point[(min_y_+4-1)%4]);
            Points2D.emplace_back(Point[(min_y_+4-2)%4]);
            Points2D.emplace_back(Point[(min_y_+4-3)%4]);
        }
    }


    void RM_BUFF_Dec::init_Img()//RGB BGR
    {
        std::vector<cv::Mat> bgr_Chs;
        cv::split(src_Img, bgr_Chs);
        if (m_Color == 0) {//Red   parallel
            //cur_Img=cv::Mat(bgr_Chs[0].size(),CV_8UC1);
            int rows = bgr_Chs[0].rows;
            int cols = bgr_Chs[0].cols;
            if (bgr_Chs[2].isContinuous() && bgr_Chs[1].isContinuous()) {
                cols = rows * cols;
                rows = 1;
            }
            for (int i = 0; i < rows; i++) {
                uchar *ptr_1 = bgr_Chs[2].ptr<uchar>(i);
                uchar *ptr_2 = bgr_Chs[1].ptr<uchar>(i);
                uchar *ptr_Out = cur_Img.ptr<uchar>(i);
                for (int j = 0; j < cols; j++) {
                    if (cv::saturate_cast<uchar>(ptr_1[j] - ptr_2[j]) > color_Thresh_red)
                        ptr_Out[j] = 255;
                    else
                        ptr_Out[j] = 0;
                }
            }
            /*
            cv::Mat tmp=bgr_Chs[0] - bgr_Chs[1];
            cv::threshold(tmp, tmp, color_Thresh_red, 255, cv::THRESH_BINARY);
            for(int i=0;i<tmp.rows;i++)
            {
                uchar* ptr_1=tmp.ptr<uchar>(i);
                uchar* ptr_2=cur_Img.ptr<uchar>(i);
                for(int j=0;j<tmp.cols;j++)
                    if(ptr_1[j]!=ptr_2[j])
                    {
                        std::cout<<"not equal\n";
                        cv::waitKey(0);
                    }
            }
             */
            //cur_Img = bgr_Chs[0] - bgr_Chs[1];//R-G
            //cv::threshold(cur_Img, cur_Img, color_Thresh_red, 255, cv::THRESH_BINARY);//ֱbinary
        } else {//Blue
            int rows = bgr_Chs[0].rows;
            int cols = bgr_Chs[0].cols;
            if (bgr_Chs[0].isContinuous() && bgr_Chs[2].isContinuous()) {
                cols = rows * cols;
                rows = 1;
            }
            for (int i = 0; i < rows; i++) {
                uchar *ptr_1 = bgr_Chs[0].ptr<uchar>(i);
                uchar *ptr_2 = bgr_Chs[2].ptr<uchar>(i);
                uchar *ptr_Out = cur_Img.ptr<uchar>(i);
                for (int j = 0; j < cols; j++) {
                    if (cv::saturate_cast<uchar>(ptr_1[j] - ptr_2[j]) > color_Thresh_blue)
                        ptr_Out[j] = 255;
                    else
                        ptr_Out[j] = 0;
                }
            }
            //cur_Img = bgr_Chs[2] - bgr_Chs[0];//B-R
            //cv::threshold(cur_Img, cur_Img, color_Thresh_blue, 255, cv::THRESH_BINARY);//ֱbinary
        }
        //cv::morphologyEx(cur_Img,cur_Img,cv::MORPH_CLOSE,element);
        //cv::dilate(cur_Img,cur_Img,element);
        //cv::imshow("befo_dilate",cur_Img);
        cv::dilate(cur_Img, cur_Img, element);
        //    cv::erode(cur_Img,cur_Img,element);
//        cv::imshow("cur_", cur_Img);
//        cv::waitKey(10);
        //cv::imshow("buff",buff_show);
    }

    cv::Mat RM_BUFF_Dec::show_Res(cv::Mat &src_Input) {
        if (isfind_Armor) {
            cv::Point2f vertices[4];
            //targets[0].self_Fit_poly(vertices);
            /*
            if(targets[0].self_Fit_rect.size.width>targets[0].self_Fit_rect.size.height)
                cv::line(src_Input,targets[0].offset_Pt+vertices[0],targets[0].offset_Pt+vertices[3],cv::Scalar(0,255,0),1);
            else
                cv::line(src_Input,targets[0].offset_Pt+vertices[0],targets[0].offset_Pt+vertices[1],cv::Scalar(0,255,0),1);
            */
//            for (int j = 0; j < 4; j++) {
//                cv::line(src_Input, targets[0].offset_Pt + targets[0].self_Fit_poly[j], targets[0].offset_Pt + targets[0].self_Fit_poly[(j + 1) % 4],
//                         cv::Scalar(0, 255, 0), 3);//3
//            }


        }
        if (isget_Roi) {
            cv::rectangle(src_Input, armor_Roi, cv::Scalar(255, 255, 255), 2);
        }
        if (x_Data.size() > 3) {
            if (!isfind_Traj) {
//                param_r = pow(pow(x_Data[0] - param_x0, 2) + pow(y_Data[0] - param_y0, 2), 0.5);
                cv::circle(src_Input, cv::Point(param_x0, param_y0), sqrt(param_r), cv::Scalar(0, 255, 0));
                cv::circle(src_Input, cv::Point(param_x0, param_y0), 3, cv::Scalar(0, 0, 255));
            }
            if (isfind_Traj) {
//                param_r = pow(pow(x_Data[0] - param_x0, 2) + pow(y_Data[0] - param_y0, 2), 0.5);
                cv::circle(src_Input, cv::Point(param_x0, param_y0), sqrt(param_r), cv::Scalar(0, 0, 255));
                cv::circle(src_Input, cv::Point(param_x0, param_y0), 3, cv::Scalar(0, 0, 255));
            }
//            for(int j=0;j<4;j++){
//                cv::line(src_Input,targets[0].offset_Pt+targets[0].self_Fit_poly[j],targets[0].offset_Pt+targets[0].self_Fit_poly[(j+1)%4],cv::Scalar(0,255,0),2);//3
//            }
        }
        return src_Input;
    }

    bool RM_BUFF_Dec::find_Armor() {
        std::vector<RM_tar_Armor>().swap(targets);//clear targets
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(cur_Img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        if (m_Color == 1) {//blue
            for (int i = 0; i < contours.size(); i++) {
                if (hierarchy[i][3] < 0 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
                    continue;
                //std::cout<<"self_Size:"<<contours[i].size()<<"  far_Size:"<<contours[static_cast<uint>(hierarchy[i][3])].size()<<"\n";//step 1
                if (contours[i].size() < armor_Pts_min || contours[i].size() > armor_Pts_max ||
                    contours[static_cast<uint>(hierarchy[i][3])].size() < far_Armor_min ||
                    contours[static_cast<uint>(hierarchy[i][3])].size() > far_Armor_max)
                    continue;  //筛选轮廓
                RM_tar_Armor cur_Armor;
                cur_Armor.self_Rect = cv::minAreaRect(contours[i]);
                double self_Ratio = MAX(cur_Armor.self_Rect.size.height, cur_Armor.self_Rect.size.width) /
                                    MIN(cur_Armor.self_Rect.size.height, cur_Armor.self_Rect.size.width);
                //std::cout << "self_Ratio::" << self_Ratio << "\n";
                if (self_Ratio < 3) {
                    cur_Armor.self_Elli = cv::fitEllipse(contours[i]);
                    cur_Armor.farther_Elli = cv::fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);
                    cur_Armor.cross_Ang = fabs(cur_Armor.self_Elli.angle - cur_Armor.farther_Elli.angle);
                    cv::Mat temp = src_Img.clone();
                    cv::ellipse(temp, cur_Armor.self_Elli, cv::Scalar(255), 2);
                    cv::ellipse(temp, cur_Armor.farther_Elli, cv::Scalar(255), 2);
                    cv::imshow("self and far ellipse", temp);
                    //std::cout << "cross_Ang:" << cur_Armor.cross_Ang << "\n";//step 2
                    if (fabs(cur_Armor.cross_Ang) < 10.0 || fabs(fabs(cur_Armor.cross_Ang) - 180.0) < 10) {
                        double far_Area = cv::contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
                        cur_Armor.farther_Rect = cv::minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);
                        double far_area_Ratio = far_Area / cur_Armor.farther_Rect.size.area();
                        double self_Ratio_far = cur_Armor.farther_Rect.size.area() / cur_Armor.self_Rect.size.area();
                        //std::cout << "far_area_Ratio:" << far_area_Ratio << "  self_Ratio_far:" << self_Ratio_far<< "\n";//step 3
                        if (fabs(far_area_Ratio - 1) < 0.3 && self_Ratio_far < 6)//find one
                        {
                            /*
                            cv::Point2f pt[4];
                            cur_Armor.self_Rect.points(pt);
                            for (int nn = 0; nn < 4; nn++) {
                                cv::line(src_Img, pt[nn], pt[(nn + 1) % 4], cv::Scalar(0, 255, 0), 2);
                            }
                             */
                            if (search_Pts(cur_Armor))
                                targets.emplace_back(cur_Armor);
                        }
                    }
                }
            }
        } else {//red
            for (int i = 0; i < contours.size(); i++) {
                if (hierarchy[i][3] < 0 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
                    continue;
                //std::cout<<"self_Size:"<<contours[i].size()<<"  far_Size:"<<contours[static_cast<uint>(hierarchy[i][3])].size()<<"\n";//step 1
                if (contours[i].size() < armor_Pts_min || contours[i].size() > armor_Pts_max ||
                    contours[static_cast<uint>(hierarchy[i][3])].size() < far_Armor_min ||
                    contours[static_cast<uint>(hierarchy[i][3])].size() > far_Armor_max)
                    continue;
                RM_tar_Armor cur_Armor;
                cur_Armor.self_Rect = cv::minAreaRect(contours[i]);
                double self_Ratio = MAX(cur_Armor.self_Rect.size.height, cur_Armor.self_Rect.size.width) /
                                    MIN(cur_Armor.self_Rect.size.height, cur_Armor.self_Rect.size.width);
                //std::cout << "self_Ratio:" << self_Ratio << "\n";
                if (self_Ratio < 3) {
                    cur_Armor.self_Elli = cv::fitEllipse(contours[i]);
                    cur_Armor.farther_Elli = cv::fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);
                    cur_Armor.cross_Ang = fabs(cur_Armor.self_Elli.angle - cur_Armor.farther_Elli.angle);
                    //std::cout << "cross_Ang:" << cur_Armor.cross_Ang << "\n";//step 2
                    if (fabs(cur_Armor.cross_Ang) < 10.0 || fabs(fabs(cur_Armor.cross_Ang) - 180.0) < 10) {
                        double far_Area = cv::contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
                        cur_Armor.farther_Rect = cv::minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);
                        double far_area_Ratio = far_Area / cur_Armor.farther_Rect.size.area();
                        double self_Ratio_far = cur_Armor.farther_Rect.size.area() / cur_Armor.self_Rect.size.area();
                        //std::cout << "far_area_Ratio:" << far_area_Ratio << "  self_Ratio_far:" << self_Ratio_far<< "\n";//step 3
                        if (fabs(far_area_Ratio - 1) < 0.3 && self_Ratio_far < 4)//find one
                        {
                            /*
                            cv::Point2f pt[4];
                            cur_Armor.self_Rect.points(pt);
                            for (int nn = 0; nn < 4; nn++) {
                                cv::line(src_Img, pt[nn], pt[(nn + 1) % 4], cv::Scalar(0, 255, 0), 2);
                            }
                             */
                            if (search_Pts(cur_Armor))
                                targets.push_back(cur_Armor);
                        }
                    }
                }
            }
        }
        //cv::imshow("real_Cont",src_Img);
        if (targets.size() == 1) {
            //todo:update order
            isfind_Armor = true;
            std::clock_t start;
            start = clock();
            //std::cout<<"angle"<<targets[0].self_Fit_rect.angle<<std::endl;
            targets[0].Time = start;
            double edgeSet[4] = {0};
            int minyIndex = 0;
            int quadrant = -1;
            if(targets[0].self_Fit_poly.empty())
            {
                return false;
            }
            for (int j = 0; j < 4; j++) {
                edgeSet[j] = (targets[0].self_Fit_poly[j].x - targets[0].self_Fit_poly[(j + 1) % 4].x) *
                             (targets[0].self_Fit_poly[j].x - targets[0].self_Fit_poly[(j + 1) % 4].x) +
                             (targets[0].self_Fit_poly[j].y - targets[0].self_Fit_poly[(j + 1) % 4].y) *
                             (targets[0].self_Fit_poly[j].y - targets[0].self_Fit_poly[(j + 1) % 4].y);
                if (targets[0].self_Fit_poly[j].y < targets[0].self_Fit_poly[minyIndex].y) minyIndex = j;
            }
            std::sort(edgeSet + 0, edgeSet + 4);
            double preEdge, nxtEdge;
            nxtEdge = (targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 1) % 4].x) *
                      (targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 1) % 4].x) +
                      (targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 1) % 4].y) *
                      (targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 1) % 4].y);
            preEdge = (targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 3) % 4].x) *
                      (targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 3) % 4].x) +
                      (targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 3) % 4].y) *
                      (targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 3) % 4].y);
            if (nxtEdge < preEdge && preEdge == edgeSet[2])
            {
                quadrant = 0;
                targets[0].angle = 180+atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 3) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 3) % 4].y)*180.f/3.1415926;
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);

            }
            if (nxtEdge > preEdge && nxtEdge == edgeSet[2])
            {
                quadrant = 1;
                targets[0].angle = atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 1) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 1) % 4].y)*180.f/3.1415926;;
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);

            }
            if (nxtEdge < preEdge && preEdge == edgeSet[3])
            {
                quadrant = 2;
                targets[0].angle = atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 3) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 3) % 4].y)*180.f/3.1415926;
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);
            }
            if (nxtEdge > preEdge && nxtEdge == edgeSet[3])
            {
                quadrant = 3;
                targets[0].angle = -180+atan2(targets[0].self_Fit_poly[minyIndex].x - targets[0].self_Fit_poly[(minyIndex + 1) % 4].x,targets[0].self_Fit_poly[minyIndex].y - targets[0].self_Fit_poly[(minyIndex + 1) % 4].y)*180.f/3.1415926;
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[minyIndex]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 1) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 2) % 4]);
                Points2D.emplace_back(targets[0].offset_Pt+targets[0].self_Fit_poly[(minyIndex + 3) % 4]);

            }
//
//
//            //std::cout << "index:" << minyIndex << " qua:" << quadrant << std::endl;
//            //std::cout << "angle:" << targets[0].angle<< " qua:" << quadrant<< std::endl;
//            //std::cout << "angle:" <<Points2D<< std::endl;
//            pnp_->solve_BUFF(receiveData_now,Points2D,targets[0].angle);
//            Points2D.clear();
                       predict_SpdC();
        }
        isget_Roi = false;
        isfind_Armor = false;
        fitSpdBeginFlag=false;
        return false;
    }




    bool RM_BUFF_Dec::predict_SpdC()
    {

        if(fitSpdBeginFlag)
        {
            preTarget=targets[0];
            fitSpdBeginFlag=false;
        }
        else
        {
            double abc[2];
            double sum=0;
            double angle = 0;
            static double diff=targets[0].angle-preTarget.angle;
            if(diff>=3.14159) diff=3.14159*2-diff;

            double delta_angle = 35;
            if(!isfitspd)
            {
                preTarget.rotSpdA=diff/((targets[0].Time-preTarget.Time)/CLOCKS_PER_SEC);
                time_Data.push_back(preTarget.Time);
                spd_Data.push_back(preTarget.rotSpdA);
                if(time_Data.size()>4)
                {
                    for(int i=0;i<time_Data.size();i++)
                    {
                        sum=sum+spd_Data[i];
                    }
                    averSpd=sum/time_Data.size();
                    isfitspd = true;
                }

            }

            if(diff<0)
            {
                angle  = targets[0].angle - delta_angle;
                if(angle<-180)
                {
                    angle +=360;
                }
            }
            else
            {
                angle = targets[0].angle + delta_angle;
                if(angle>180)
                {
                    angle -=360;
                }
            }
            //targets[0].angle = angle;
            //std::cout<<angle<<std::endl;
            //std::cout<<"angle:"<<angle<<"targets[0].angle"<<targets[0].angle<<std::endl;
            pnp_->solve_BUFF(receiveData_now,Points2D,targets[0].angle);
            Points2D.clear();
            targets[0].self_Fit_poly.clear();


        }


    }



    bool RM_BUFF_Dec::fit_SpdC()
    {
        if(isfind_Traj)
        {
            //static cv::Mat last_A = pnp_->solve_BUFF_A(receiveData_now,TF_3D);
            cv::Mat A_ORI_invert = cv::Mat::zeros(4,4,CV_64FC1);
            cv::Mat predictCenter_ = (cv::Mat_<double>(3, 1) <<
                                                             param_x0, param_y0, 1.f);
//            receiveData receiveData_;
//            receiveData_.pitch = receiveData_now.pitch - firstpitch;
//            receiveData_.yaw = receiveData_now.yaw - firstyaw;
//            A = pnp_->solve_BUFF_A(receiveData_now,TF_3D);
//            cv::Mat Point_temp = cv::Mat::zeros(3,1,CV_8SC1);
//            cv::invert(A_ORI,A_ORI_invert,cv::DECOMP_SVD);
//            Point_temp = A*A_ORI_invert*predictCenter_;
//            std::cout<<"dx"<<Point_temp.at<double>(0)<<"dy"<<Point_temp.at<double>(1)<<std::endl;
//            param_x0 = Point_temp.at<double>(0);
//            param_y0 = Point_temp.at<double>(1);
//            std::cout<<"x0"<<param_x0<<"y0"<<param_y0<<std::endl;
//            std::cout<<"A"<<A - A_ORI<<std::endl;
            int quadrant;
            double buffx=targets[0].center.x-param_x0;
            double buffy=targets[0].center.y-param_y0;
            if(buffx>0&&buffy>=0) quadrant=0;
            if(buffx<=0&&buffy>=0) quadrant=1;
            if(buffx<0&&buffy<=0) quadrant=2;
            if(buffx>=0&&buffy<0) quadrant=3;
            targets[0].rotAng=atan2(targets[0].center.y-param_y0,targets[0].center.x-param_x0);
            if(fitSpdBeginFlag)
            {
                preTarget=targets[0];
                fitSpdBeginFlag=false;
            }
            else
            {
                double diff;
                double abc[2];
                double sum=0;
                static double averSpd;
                diff=targets[0].rotAng-preTarget.rotAng;
                if(diff>=3.14159) diff=3.14159*2-diff;
                preTarget.rotSpdA=diff/((targets[0].Time-preTarget.Time)/CLOCKS_PER_SEC);
                time_Data.push_back(preTarget.Time);
                spd_Data.push_back(preTarget.rotSpdA);
                if(time_Data.size()>3&&time_Data.size()<30)
                {
                    for(int i=0;i<time_Data.size();i++)
                    {
                        sum=sum+spd_Data[i];
                    }
                    averSpd=sum/time_Data.size();
                }
                double newSpd=averSpd;
                targets[0].rotSpdA=newSpd;
                double predictAng=targets[0].rotAng+newSpd*1;
                double angDiff=newSpd*3;
                int o_quadrant=quadrant;
                while(angDiff>3.14159/2)
                {
                    angDiff=angDiff-3.14159/2;
                    quadrant++;
                }
                if((targets[0].rotAng+angDiff)>0&&o_quadrant==3) quadrant++;
                if((targets[0].rotAng+angDiff)>3.14159&&o_quadrant==1) quadrant++;
                if((targets[0].rotAng+angDiff)>-3.14159/2&&o_quadrant==2) quadrant++;
                if((targets[0].rotAng+angDiff)>3.14159/2&&o_quadrant==0) quadrant++;
                quadrant=quadrant%4;
                double predictX,predictY;
                predictX=(cos(predictAng))*sqrt(param_r);
                predictY=(sin(predictAng))*sqrt(param_r);
//                if(quadrant==1) predictX=-predictX;
//                if(quadrant==2)
//                {
//                    predictX=-predictX;
//                    predictY=-predictY;
//                }
//                if(quadrant==3) predictY=-predictY;
//                predictX=predictX+param_x0;
//                predictY=predictY+param_y0;
                predictCenter=cv::Point2f(predictX,predictY);
//                cv::Mat predictCenter_ = (cv::Mat_<float>(3, 1) <<
//                        predictX, predictY, 1);
//                A = pnp_->solve_BUFF_A(receiveData_now,TF_3D);
//                cv::Mat Point_temp = A*predictCenter_;


                double x_small = sqrt(param_r)/0.7*0.1075;
                double y_small = sqrt(param_r)/0.7*0.0625;
                cv::Point2f temp1,temp2;
                temp1.x = cos(predictAng)*(sqrt(param_r)-y_small);
                temp1.y = sin(predictAng)*(sqrt(param_r)-y_small);
                temp2.x = cos(predictAng)*(sqrt(param_r)+y_small);
                temp2.y = sin(predictAng)*(sqrt(param_r)+y_small);
                predict[0].Point_pre.x = temp1.x + sin(predictAng)*x_small+param_x0;
                predict[0].Point_pre.y = temp1.y - cos(predictAng)*x_small+param_y0;
                predict[1].Point_pre.x = temp1.x - sin(predictAng)*x_small+param_x0;
                predict[1].Point_pre.y = temp1.y + cos(predictAng)*x_small+param_y0;
                predict[2].Point_pre.x = temp2.x - sin(predictAng)*x_small+param_x0;
                predict[2].Point_pre.y = temp2.y + cos(predictAng)*x_small+param_y0;
                predict[3].Point_pre.x = temp2.x + sin(predictAng)*x_small+param_x0;
                predict[3].Point_pre.y = temp2.y - cos(predictAng)*x_small+param_y0;
//                temp1.x += +param_x0;
//                temp1.y += +param_y0;
//                temp2.x += param_x0;
//                temp2.y += param_y0;
//                cv::circle(real_Src,  predict[0].Point_pre, 4, cv::Scalar(0, 255, 0));
//                cv::circle(real_Src, predict[1].Point_pre, 4, cv::Scalar(255, 0, 0));
//                      for(int j = 0; j < 4; j++) {
//                            cv::line(real_Src, predict[j].Point_pre, predict[(j + 1) % 4].Point_pre,
//                                     cv::Scalar(0, 255, 0), 1);//3
//                        }
//                cv::imshow("ss",real_Src);
//                std::cout<<"preX "<<predictCenter.x<<" Y "<<predictCenter.y<<std::endl;
//                std::cout<<"spd"<<targets[0].rotSpdA<<std::endl;
//                std::cout<<"abc0"<<abc[0]<<" abc1 "<<abc[1]<<std::endl;
                preTarget=targets[0];
//                for(int j = 0; j < 4; j++) {
//                    cv::line(real_Src, predict[j].Point_pre, predict[(j + 1) % 4].Point_pre,
//                             cv::Scalar(255, 0, 0), 1);//3
//                }
//                cv::imshow("ss",real_Src);
                ispredict = true;


                cv::Point2f vertices[4];

                for (int i = 0; i < 4; i++) {
                    vertices[i] = predict[i].Point_pre;
                    Points2D.emplace_back(vertices[i]+targets[0].offset_Pt);
                }
                //pnp_->solve_BUFF(receiveData_now,Points2D);
                Points2D.clear();
            }
        }
    }

    bool RM_BUFF_Dec::fit_SpdC_3D()
    {
        if(isfind_Traj)
        {
            if(Target_3D_.Point.z()>circle_3D_.Cz)
            {
                Target_3D_.angle =  targets[0].self_Fit_rect.angle/180.f*3.1415926;
            }
            else
            {
                Target_3D_.angle =  targets[0].self_Fit_rect.angle/180.f*3.1415926;
            }
            std::cout<<Target_3D_.angle<<std::endl;

            if(fitSpdBeginFlag)
            {
                Target_3D_pre=Target_3D_;
                fitSpdBeginFlag=false;
            }
            else
            {
                double diff;
                double sum=0;
                static double averSpd;
                diff=Target_3D_.angle-Target_3D_pre.angle;
                if(diff>=3.14159) diff=3.14159*2-diff;
                Target_3D_.speed=diff/((Target_3D_.Time-Target_3D_pre.Time)/CLOCKS_PER_SEC);
                time_Data.push_back(Target_3D_.Time);
                spd_Data.push_back(Target_3D_.angle);
                if(time_Data.size()>3&&time_Data.size()<30)
                {
                    for(int i=0;i<time_Data.size();i++)
                    {
                        sum=sum+spd_Data[i];
                    }
                    averSpd=sum/time_Data.size();
                }
                Target_3D_.speed = averSpd;
                Target_3D_predict.angle = Target_3D_.angle+averSpd*0.5;
//                cv::Mat armor_temp = (cv::Mat_<float>(3, 1)
//                        <<circle_3D_.radius*cos(Target_3D_predict.angle),0,circle_3D_.radius*sin(Target_3D_predict.angle));
                cv::Mat R  = (cv::Mat_<float>(3, 3)<<0,circle_3D_.a,0
                        ,0,circle_3D_.b,0
                        ,0,circle_3D_.c,0);
                Eigen::Matrix3d Rotate_M = Eigen::Matrix3d::Identity();
                cv::cv2eigen(R,Rotate_M);
                Sophus::SO3<double> rotate(Rotate_M);
                Eigen::Vector3d translate(circle_3D_.Cx,circle_3D_.Cy,-circle_3D_.Cz);
                TF_3D = Sophus::SE3<double>(rotate,translate);

                Eigen::Matrix3d rotation_matrix3;
                rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
                Sophus::SE3<double> armor_temp = Sophus::SE3<double>(rotation_matrix3,Eigen::Vector3d(circle_3D_.radius*cos(Target_3D_predict.angle),0,circle_3D_.radius*sin(Target_3D_predict.angle)));


                armor_temp *=TF_3D;
//                transform_->trans_BUFF(armor_temp,receiveData_now);
            }

        }
    }

    bool RM_BUFF_Dec::search_Pts(RM_tar_Armor &cur_Armor)
    {
        if(m_Color==1) {//blue
            cv::RotatedRect cur_Rect = cur_Armor.self_Rect;
            cur_Rect.size.width = 1.5 * cur_Rect.size.width;
            cur_Rect.size.height = 1.5 * cur_Rect.size.height;
            cv::Point2f vertices[4];
            cur_Rect.points(vertices);
            std::vector<cv::Point2f> pts;
            for (int i = 0; i < 4; i++)
                pts.push_back(2 * vertices[i]);//scale
            cv::Rect cur_Roi = cv::boundingRect(pts);
            /*
            if(cur_Roi.tl().x<0)
                cur_Roi.x=0;
            if(cur_Roi.tl().y<0)
                cur_Roi.y=0;
            if(cur_Roi.br().x>=real_Src.cols)
                cur_Roi.x=real_Src.cols-1-cur_Roi.width;
            if(cur_Roi.br().y>=real_Src.rows)
                cur_Roi.y=real_Src.rows-1-cur_Roi.height;
            */
            cur_Roi &= cv::Rect(0,0,real_Src.cols,real_Src.rows);
            cur_Armor.offset_Pt = cur_Roi.tl();//offset point
            cv::Mat cont_Img;
            real_Src(cur_Roi).copyTo(cont_Img);

            std::vector<cv::Mat> chs;
            cv::split(cont_Img,chs);
            //cont_Img=chs[2]-chs[0];
            //cv::threshold(cont_Img, cont_Img, 30, 255, CV_THRESH_BINARY);
            cont_Img=cv::Mat(cont_Img.size(),CV_8UC1);
            int rows=cont_Img.rows;
            int cols=cont_Img.cols;
            if(chs[0].isContinuous() && chs[2].isContinuous())
            {
                cols*=rows;
                rows=1;
            }
            for(int i=0;i<rows;i++)
            {
                uchar* ptr_1=chs[0].ptr<uchar>(i);
                uchar* ptr_2=chs[2].ptr<uchar>(i);
                uchar* ptr_Out=cont_Img.ptr<uchar>(i);
                for(int j=0;j<cols;j++)
                {
                    if(cv::saturate_cast<uchar>(ptr_1[j]-ptr_2[j])>30)
                        ptr_Out[j]=255;
                    else
                        ptr_Out[j]=0;
                }
            }

//        cv::cvtColor(cont_Img, cont_Img, cv::COLOR_BGR2GRAY);
//        cv::threshold(cont_Img, cont_Img, 30, 255, cv::THRESH_BINARY);
//        cv::morphologyEx(cont_Img, cont_Img, cv::MORPH_CLOSE, ele4blue);
//
//            cv::imshow("cont", cont_Img);
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(cont_Img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            for (int i = 0; i < contours.size(); i++) {
                //std::cout<<"size ratio:"<<cv::contourArea(contours[i])/(4*cur_Armor.self_Rect.size.area())<<"\n";
                if (fabs(cv::contourArea(contours[i]) / (4 * cur_Armor.self_Rect.size.area()) - 1) < 0.5) {
                    cur_Armor.self_Fit_rect = cv::minAreaRect(contours[i]);
                    double ratio_1 = MAX(cur_Armor.self_Rect.size.width, cur_Armor.self_Rect.size.height) /
                                     MIN(cur_Armor.self_Rect.size.width, cur_Armor.self_Rect.size.height);
                    double ratio_2 = MAX(cur_Armor.self_Fit_rect.size.width, cur_Armor.self_Fit_rect.size.height) /
                                     MIN(cur_Armor.self_Fit_rect.size.width, cur_Armor.self_Fit_rect.size.height);
                    if (fabs(ratio_2 / ratio_1 - 1) < 0.2) {
                        cv::approxPolyDP(cv::Mat(contours[static_cast<uint>(hierarchy[i][3])]), cur_Armor.self_Fit_poly, 10, true);
                        std::cout<<cur_Armor.self_Fit_poly<<std::endl;
                        return true;
                    }
                }
            }
            return false;
        }
        else{//red
            cv::RotatedRect cur_Rect = cur_Armor.self_Rect;
            cur_Rect.size.width = 1.5 * cur_Rect.size.width;
            cur_Rect.size.height = 1.5 * cur_Rect.size.height;
            cv::Point2f vertices[4];
            cur_Rect.points(vertices);
            std::vector<cv::Point2f> pts;
            for (int i = 0; i < 4; i++)
                pts.push_back(2 * vertices[i]);//scale
            cv::Rect cur_Roi = cv::boundingRect(pts);
            /*
            if(cur_Roi.tl().x<0)
                cur_Roi.x=0;
            if(cur_Roi.tl().y<0)
                cur_Roi.y=0;
            if(cur_Roi.br().x>=real_Src.cols)
                cur_Roi.x=real_Src.cols-1-cur_Roi.width;
            if(cur_Roi.br().y>=real_Src.rows)
                cur_Roi.y=real_Src.rows-1-cur_Roi.height;
                */
            //test
            cur_Roi &= cv::Rect(0,0,real_Src.cols,real_Src.rows);
            //test
            cur_Armor.offset_Pt = cur_Roi.tl();//offset point
            cv::Mat cont_Img;
            real_Src(cur_Roi).copyTo(cont_Img);

            std::vector<cv::Mat> chs;
            cv::split(cont_Img,chs);
            cont_Img=cv::Mat(cont_Img.size(),CV_8UC1);
            int rows=cont_Img.rows;
            int cols=cont_Img.cols;
            if(chs[2].isContinuous() && chs[1].isContinuous())
            {
                cols*=rows;
                rows=1;
            }
            for(int i=0;i<rows;i++)
            {
                uchar* ptr_1=chs[2].ptr<uchar>(i);
                uchar* ptr_2=chs[1].ptr<uchar>(i);
                uchar* ptr_Out=cont_Img.ptr<uchar>(i);
                for(int j=0;j<cols;j++)
                {
                    if(cv::saturate_cast<uchar>(ptr_1[j]-ptr_2[j])>30)
                        ptr_Out[j]=255;
                    else
                        ptr_Out[j]=0;
                }
            }

//        cv::cvtColor(cont_Img, cont_Img, cv::COLOR_BGR2GRAY);
//        cv::threshold(cont_Img, cont_Img, 30, 255, cv::THRESH_BINARY);
//        cv::morphologyEx(cont_Img, cont_Img, cv::MORPH_CLOSE, ele4red);

            //cont_Img=chs[0]-chs[1];
            //cv::threshold(cont_Img, cont_Img, 30, 255, CV_THRESH_BINARY);
            //cv::morphologyEx(cont_Img, cont_Img, cv::MORPH_CLOSE, ele4red);
            cv::dilate(cont_Img,cont_Img,ele4red);
//        cv::imshow("cont", cont_Img);
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(cont_Img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            std::vector<cv::RotatedRect> rects;
            for (int i = 0; i < contours.size(); i++) {
                //std::cout<<"size ratio:"<<cv::contourArea(contours[i])/(4*cur_Armor.self_Rect.size.area())<<"\n";
                double cur_Ratio=cv::contourArea(contours[i]) / (4 * cur_Armor.self_Rect.size.area());
                if (cur_Ratio<1.2 && cur_Ratio>0.6) {
                    cur_Armor.self_Fit_rect = cv::minAreaRect(contours[i]);
                    double ratio_1 = MAX(cur_Armor.self_Rect.size.width, cur_Armor.self_Rect.size.height) /
                                     MIN(cur_Armor.self_Rect.size.width, cur_Armor.self_Rect.size.height);
                    double ratio_2 = MAX(cur_Armor.self_Fit_rect.size.width, cur_Armor.self_Fit_rect.size.height) /
                                     MIN(cur_Armor.self_Fit_rect.size.width, cur_Armor.self_Fit_rect.size.height);
                    if (fabs(ratio_2 / ratio_1 - 1) < 0.3) {
                        cv::approxPolyDP(cv::Mat(contours[static_cast<uint>(hierarchy[i][3])]), cur_Armor.self_Fit_poly, 10, true);
                        //std::cout<<"poly:"<<cur_Armor.self_Fit_poly<<std::endl;
                        return true;
                    }
                }
            }
            return false;
        }
    }


//void RM_BUFF_Dec::cal_Ang(short &pitch, short &yaw)//todo pitch->un-down yaw->left-right   positive:up left
//{
//    cv::Point2f tar_Pos=targets[0].offset_Pt+targets[0].self_Fit_rect.center;
//    pitch=100.0*180.0*atan2(fabs(tar_Pos.y-cam_V0),cam_F)/CV_PI;
//    yaw=100.0*180.0*atan2(fabs(tar_Pos.x-cam_U0),cam_F)/CV_PI;
//    if(tar_Pos.y>cam_V0)//down
//        pitch*=-1;
//    if(tar_Pos.x>cam_U0)
//        yaw*=-1;
//    //todo:judge right direction
//}

    void RM_BUFF_Dec::cal_Ang(receiveData receiveData_) {
        //std::cout<<"imu_pitch"<<imu_pitch<<"imu_yaw"<<imu_yaw<<std::endl;
        cv::Point2f vertices[4];

        for (int i = 0; i < 4; i++) {
            vertices[i] = predict[i].Point_pre;
            Points2D.emplace_back(vertices[i]+targets[0].offset_Pt);
        }
//        if(vertices[0].x==0)
//        {
//            return;
//        }
//        for (int j = 0; j < 4; j++) {
//            cv::line(real_Src,  vertices[j], vertices[(j + 1) % 4],
//                     cv::Scalar(0, 255, 0), 5);//3
//        }
//        cv::line(real_Src, cv::Point(0,0),  cv::Point(100,100),cv::Scalar(0, 255, 0), 5);//3

        //pnp_->solve_BUFF(receiveData_,Points2D);
        Points2D.clear();
    }

}
