#ifndef PROCESSPIPE_H
#define PROCESSPIPE_H

#include <opencv2/opencv.hpp>
#include "ThreadCam.h"
#include "MJPGthread.h"
#include "yolo_v2_class.hpp"	        // imported functions from .so

//----------------------------------------------------------------------------------
class TProcessPipe
{
private:
    int CamNr;
    int  Width;                                     //width and height of the frame we process. see config.json
    int  Height;                                    //need not be the camera resolution (smaller will speed up calcus)
    int  MJpeg;
    ThreadCam  *CamGrb;
    MJPGthread *MJ;
    cv::Rect    DnnRect;                            //roi used by pre-DNN models (not the ALPR darknet models)
    cv::Mat     MatEmpty;                           //gray (empty) frame
    bool        UseNetRect;                         //quick check whether we must use NetRect
    bool        RdyNetRect;                         //quick check if NetRect is adapted
protected:
    void ProcessDNN(cv::Mat& frame);
    void DrawObjects(cv::Mat& frame, const std::vector<bbox_t>& boxes);
public:
    cv::Mat    MatAr;                               //frame received from CamGrb

    TProcessPipe(void);
    virtual ~TProcessPipe(void);
    void Init(const int Nr);
    void StartThread(void);
    void StopThread(void);
    void ExecuteCam(void);
};
//----------------------------------------------------------------------------------
#endif // PROCESSPIPE_H

