#include "General.h"
#include "ProcessPipe.h"
#include "net.h"
#include "Tjson.h"
#include "yolo_v2_class.hpp"	        // imported functions from .so

using namespace std;

extern Tjson    Js;
extern Detector YoloV4net;
//---------------------------------------------------------------------------
static const char* class_names[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};
//---------------------------------------------------------------------------
// TProcessPipe
//---------------------------------------------------------------------------
TProcessPipe::TProcessPipe(void): DnnRect(0,0,WORK_WIDTH,WORK_HEIGHT)
{
    CamNr  = -1;
    MJpeg  = 0;
    CamGrb = NULL;
    MJ     = NULL;
}
//---------------------------------------------------------------------------
TProcessPipe::~TProcessPipe(void)
{
    if(MJ     != NULL) delete MJ;
    if(CamGrb != NULL) delete CamGrb;
}
//---------------------------------------------------------------------------
void TProcessPipe::Init(const int Nr)
{
    Tjson TJ;

    TJ.Jvalid = true;       //force the json to be valid, else no GetSetting() will work
    CamNr     = Nr;
    Width     = Js.WorkWidth;
    Height    = Js.WorkHeight;

    CamGrb = new ThreadCam(Width,Height);

    MatEmpty = cv::Mat(Height, Width, CV_8UC3, cv::Scalar(128,128,128));

    MatEmpty.copyTo(MatAr);

    json Jstr=Js.j["STREAM_"+std::to_string(CamNr+1)];

    TJ.GetSetting(Jstr,"MJPEG_PORT",MJpeg);
    if(MJpeg>7999){
        MJ = new MJPGthread();
        MJ->Init(MJpeg);
    }

    //get DNN rect
    if(!TJ.GetSetting(Jstr["DNN_Rect"],"x_offset",DnnRect.x))     std::cout << "Using default value" << std::endl;
    if(!TJ.GetSetting(Jstr["DNN_Rect"],"y_offset",DnnRect.y))     std::cout << "Using default value" << std::endl;
    if(!TJ.GetSetting(Jstr["DNN_Rect"],"width",DnnRect.width))    std::cout << "Using default value" << std::endl;
    if(!TJ.GetSetting(Jstr["DNN_Rect"],"height",DnnRect.height))  std::cout << "Using default value" << std::endl;

    //check DNN rect with our camera
    if(DnnRect.x     <     0) DnnRect.x     = 0;
    if(DnnRect.width > Width) DnnRect.width = Width;
    if(DnnRect.y     <     0) DnnRect.y     = 0;
    if(DnnRect.height>Height) DnnRect.height = Height;
    if((DnnRect.x+DnnRect.width ) >  Width) DnnRect.x=Width -DnnRect.width;
    if((DnnRect.y+DnnRect.height) > Height) DnnRect.y=Height-DnnRect.height;

    UseNetRect = (DnnRect.x!= 0 || DnnRect.y!=0 || DnnRect.width!=Width || DnnRect.height!=Height);
    RdyNetRect = false;
}
//---------------------------------------------------------------------------
void TProcessPipe::StartThread(void)
{
    std::string Jstr="STREAM_"+std::to_string(CamNr+1);

    CamGrb->Start(Js.j[Jstr]);
}
//---------------------------------------------------------------------------
void TProcessPipe::StopThread(void)
{
    CamGrb->Quit();
}
//----------------------------------------------------------------------------------------
void TProcessPipe::ExecuteCam(void)
{
    int x,y,b;
    int Wd,Ht;
    int Wn,Hn;
    int Wf,Hf;
    cv::Mat Mdst;
    bool Success;
    float Ratio=1.0;

    if(CamGrb->PipeLine=="") return;

    Success = CamGrb->GetFrame(MatAr);

    if(Success){
        //width or height may differ. (depends on the camera settings)
        Wd=MatAr.cols;
        Ht=MatAr.rows;

        if(Wd!=Width || Ht!=Height){
            if(Wd > Ht){
                Ratio = (float)Width / Wd;
                Wn = Width;
                Hn = Ht * Ratio;
            }
            else{
                Ratio = (float)Height / Ht;
                Hn = Height;
                Wn = Wd * Ratio;
            }
            resize(MatAr, Mdst, cv::Size(Wn,Hn), 0, 0, cv::INTER_LINEAR); // resize to 640xHn or Wnx480 resolution
            //now work with the Mdst frame
            ProcessDNN(Mdst);

            //next place it in a proper 640x480 frame
            MatEmpty.copyTo(MatAr);
            if(Hn<Height){
                Hf=(Height-Hn)/2;
                Mdst.copyTo(MatAr(cv::Rect(0, Hf, Width, Mdst.rows)));
            }
            else{
                Wf=(Width-Wn)/2;
                Mdst.copyTo(MatAr(cv::Rect(Wf, 0, Mdst.cols, Height)));
            }
        }
        else{
            //width and height are equal the view, work direct on MatAr
            ProcessDNN(MatAr);
        }
    }
    else MatEmpty.copyTo(MatAr);

    //place name at the corner
    cv::Size label_size = cv::getTextSize(CamGrb->CamName.c_str(), cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &b);
    x = Width  - label_size.width - 4;
    y = Height - b - 4;
    cv::putText(MatAr, CamGrb->CamName.c_str(), cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0));

    //send the result to the socket
    if(MJpeg>0){
        MJ->Send(MatAr);
    }

}
//----------------------------------------------------------------------------------
void TProcessPipe::ProcessDNN(cv::Mat& frame)
{
    cv::Mat DNNframe;
    std::vector<bbox_t> objects;

    if(UseNetRect){
        //initial check, no need to check every frame because all variables will never change any more
        if(!RdyNetRect){
            if(DnnRect.x     <         0) DnnRect.x     = 0;
            if(DnnRect.width >frame.cols) DnnRect.width = frame.cols;
            if(DnnRect.y     <         0) DnnRect.y     = 0;
            if(DnnRect.height>frame.rows) DnnRect.height = frame.rows;
            if((DnnRect.x+DnnRect.width ) > frame.cols) DnnRect.x=frame.cols -DnnRect.width;
            if((DnnRect.y+DnnRect.height) > frame.rows) DnnRect.y=frame.rows-DnnRect.height;
            RdyNetRect = true;
        }
        frame(DnnRect).copyTo(DNNframe);

        objects = YoloV4net.detect(DNNframe,0.4);

        //shift the outcome to its position in the original frame
        for(size_t i = 0; i < objects.size(); i++) {
            objects[i].x += DnnRect.x;
            objects[i].y += DnnRect.y;
        }
    }
    else{
        objects = YoloV4net.detect(frame,0.4);
    }

    objects = YoloV4net.tracking_id(objects);

    DrawObjects(frame, objects);
}
//---------------------------------------------------------------------------
void TProcessPipe::DrawObjects(cv::Mat& frame, const std::vector<bbox_t>& boxes)
{
    if(Js.PrintOnCli){
        std::cout << CamGrb->CamName << endl;
        for(size_t i = 0; i < boxes.size(); i++){
            std::cout <<boxes[i].prob<<" "<< class_names[boxes[i].obj_id] << std::endl;
        }
    }

    if(Js.PrintOnRender || Js.MJPEG_Port>7999){
        for(size_t i = 0; i < boxes.size(); i++) {

            char text[256];
            if(boxes[i].track_id > 0 )
                sprintf(text, "%i | %s %.1f%%", boxes[i].track_id, class_names[boxes[i].obj_id], boxes[i].prob * 100);
            else
                sprintf(text, "%s %.1f%%", class_names[boxes[i].obj_id], boxes[i].prob * 100);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            int x = boxes[i].x;
            int y = boxes[i].y - label_size.height - baseLine;
            if (y < 0) y = 0;
            if (x + label_size.width > frame.cols) x = frame.cols - label_size.width;

            cv::rectangle(frame, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                          cv::Scalar(255, 255, 255), -1);

            cv::putText(frame, text, cv::Point(x, y + label_size.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

            cv::rectangle (frame, cv::Point(boxes[i].x, boxes[i].y),
                                  cv::Point(boxes[i].x+boxes[i].w, boxes[i].y+boxes[i].h), cv::Scalar(255,0,0));
        }
    }
}
//----------------------------------------------------------------------------------

