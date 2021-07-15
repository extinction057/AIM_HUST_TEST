//
// Created by angrypsyduck on 2021/9/12.
//

#include<tool_cvex.h>
namespace cvex {
    void subtractBGR(cv::Mat &src, cv::Mat &dst, int color, float threshold) {
        uchar *pdata = (uchar *) src.data;
        uchar *qdata = (uchar *) dst.data;
        int srcData = src.rows * src.cols;
        switch (color) {
            case 0:
                color = 2;
                break;
            case 1:
                color = 0;
                break;
            default:
                break;
        }
        for (int i = 0; i < srcData; i++) {
            if (*(pdata + color) - *(pdata + (color == 0 ? 2 : 0)) > threshold)
                *qdata = 255;
            pdata += 3;
            qdata++;
        }
    }

    void inRangeHSV(cv::Mat &src,cv::Mat &dst,int color) {
//#define HSVIMG
        uchar *pdata = (uchar *) src.data;
        uchar *qdata = (uchar *) dst.data;
#ifdef HSVIMG
        cv::Mat test=cv::Mat::zeros(src.rows,src.cols,CV_8UC3);
        uchar *testdata = (uchar *) test.data;
#endif
        int srcData = src.rows * src.cols;
        for (int i = 0; i < srcData; i++) {
            int B = *(pdata + 0);
            int G = *(pdata + 1);
            int R = *(pdata + 2);
            int V, min;
            float S,H;
            if (B > G) {
                if (B > R) V = B;
                else
                    V = R;
            } else {
                if (G > R) V = G;
                else
                    V = R;
            }
            if (B < G) {
                if (B < R) min = B;
                else
                    min = R;
            } else {
                if (G < R) min = G;
                else
                    min = R;
            }
            float f = V - min;
            if (V != 0) S = f / V;
            else
                S = 0;
            S = S * 255;
            if (f != 0) {
                if (V == R) H = 60 * (G - B) / f;
                if (V == G) H = 120 + 60 * (B - R) / f;
                if (V == B) H = 240 + 60 * (R - G) / f;
                if (H < 0) H = H + 360;
                H = H / 2;
            } else
                H = 0;
#ifdef HSVIMG
            *(testdata+0)=H+0.5;
            *(testdata+1)=S+0.5;
            *(testdata+2)=V;
#endif
            if(color==0) {
                if (((H > 0 && H < 10) || ((H > 156) && (H < 180))) &&
                    (S >= 43 && S <= 255) &&
                    (V >= 46 && V <= 255)) *qdata=255;
            }
            if(color==1){
                if ((H > 100 && H < 124) &&
                    (S >= 43 && S <= 255) &&
                    (V >= 46 && V <= 255)) *qdata=255;
            }
            pdata += 3;
            qdata++;
#ifdef HSVIMG
            testdata+=3;
#endif
        }
#ifdef HSVIMG
        imshow("HSVi",test);
#endif
    }
}
