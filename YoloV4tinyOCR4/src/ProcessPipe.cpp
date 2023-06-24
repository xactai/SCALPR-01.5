#include "General.h"
#include "ProcessPipe.h"
#include "net.h"
#include "Tjson.h"
#include "yolo_v2_class.hpp"	        // imported functions from .so

using namespace std;

extern Tjson    Js;
extern Detector YoloV4net;

//---------------------------------------------------------------------------
static const unsigned char MarkColor[6][3] = {{0, 137, 114}, {0, 114, 137},
                                               {137, 0, 114}, {114, 0, 137},
                                               {137, 114, 0}, {114, 137, 0}};
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
static cv::Mat MorphKernal = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 11, 11 ), cv::Point( 5, 5 ) );
//---------------------------------------------------------------------------
// TProcessPipe
//---------------------------------------------------------------------------
TProcessPipe::TProcessPipe(void): DnnRect(0,0,WORK_WIDTH,WORK_HEIGHT)
{
    CamNr  = -1;
    MJpeg  = 0;
    CamGrb = NULL;
    MJ     = NULL;
    Trace.VehicleRect = DnnRect;
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
    double d;

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

    //get Vehicle rect
    if(TJ.GetSetting(Jstr["VEHICLE_Rect"],"left",d))  { Trace.VehicleRect.x=(int)(d*DnnRect.width) +DnnRect.x; }
    else                                              { Trace.VehicleRect.x=                        DnnRect.x; }
    if(TJ.GetSetting(Jstr["VEHICLE_Rect"],"top",d))   { Trace.VehicleRect.y=(int)(d*DnnRect.height)+DnnRect.y; }
    else                                              { Trace.VehicleRect.y=                        DnnRect.y; }
    if(TJ.GetSetting(Jstr["VEHICLE_Rect"],"right",d)) { Trace.VehicleRect.width= (int)((1.0-d)*DnnRect.width); }
    else                                              { Trace.VehicleRect.width=               DnnRect.width; }
    if(TJ.GetSetting(Jstr["VEHICLE_Rect"],"bottom",d)){ Trace.VehicleRect.height=(int)((1.0-d)*DnnRect.height);}
    else                                              { Trace.VehicleRect.height=              DnnRect.height;}

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
        cv::Mat FrameMJPEG(Js.MJPEG_Height, Js.MJPEG_Width, CV_8UC3);
        cv::resize(MatAr,FrameMJPEG,FrameMJPEG.size(),0,0);
        MJ->Send(FrameMJPEG);
    }

}
//----------------------------------------------------------------------------------
void TProcessPipe::ProcessDNN(cv::Mat& frame)
{
    TObject ob;
    cv::Mat DNNframe;
    std::vector<bbox_t> BoxObjects;
    std::vector<TObject> Objects;

    if(UseNetRect){
        //initial check, no need to check every frame because all variables will never change any more
        if(!RdyNetRect){
            if(DnnRect.x     <         0) DnnRect.x     = 0;
            if(DnnRect.width >frame.cols) DnnRect.width = frame.cols;
            if(DnnRect.y     <         0) DnnRect.y     = 0;
            if(DnnRect.height>frame.rows) DnnRect.height = frame.rows;
            if((DnnRect.x+DnnRect.width ) > frame.cols) DnnRect.x=frame.cols -DnnRect.width;
            if((DnnRect.y+DnnRect.height) > frame.rows) DnnRect.y=frame.rows-DnnRect.height;

            if(Trace.VehicleRect.x                          < DnnRect.x               ) Trace.VehicleRect.x      = DnnRect.x;
            if(Trace.VehicleRect.x+Trace.VehicleRect.width  > DnnRect.x+DnnRect.width ) Trace.VehicleRect.width  = DnnRect.x+DnnRect.width-Trace.VehicleRect.x;
            if(Trace.VehicleRect.y                          < DnnRect.y               ) Trace.VehicleRect.y      = DnnRect.y;
            if(Trace.VehicleRect.y+Trace.VehicleRect.height > DnnRect.y+DnnRect.height) Trace.VehicleRect.height = DnnRect.y+DnnRect.height-Trace.VehicleRect.y;;
            RdyNetRect = true;
        }
        frame(DnnRect).copyTo(DNNframe);

        BoxObjects = YoloV4net.detect(DNNframe);

        //shift the outcome to its position in the original frame
        for(size_t i = 0; i < BoxObjects.size(); i++) {
            BoxObjects[i].x += DnnRect.x;
            BoxObjects[i].y += DnnRect.y;
        }
    }
    else{
        //initial check, no need to check every frame because all variables will never change any more
        if(!RdyNetRect){
            if(Trace.VehicleRect.x                          < DnnRect.x               ) Trace.VehicleRect.x      = DnnRect.x;
            if(Trace.VehicleRect.x+Trace.VehicleRect.width  > DnnRect.x+DnnRect.width ) Trace.VehicleRect.width  = DnnRect.x+DnnRect.width-Trace.VehicleRect.x;
            if(Trace.VehicleRect.y                          < DnnRect.y               ) Trace.VehicleRect.y      = DnnRect.y;
            if(Trace.VehicleRect.y+Trace.VehicleRect.height > DnnRect.y+DnnRect.height) Trace.VehicleRect.height = DnnRect.y+DnnRect.height-Trace.VehicleRect.y;;
            RdyNetRect = true;
        }

        BoxObjects = YoloV4net.detect(frame);
    }

    //get the tracking ids
    BoxObjects = YoloV4net.tracking_id(BoxObjects,true,20,100);

    //file the Objects with the BoxObjects
    for(size_t i = 0; i < BoxObjects.size(); i++) {
        ob=BoxObjects[i];
        Objects.push_back(ob);
    }

    GetPlatePosition(frame, Objects);

    DrawObjects(frame, Objects);

}
//---------------------------------------------------------------------------
// Discrete OpenCV method = 9 mSec | DarkNet = 67 mSec
//---------------------------------------------------------------------------
bool TProcessPipe::GetPlatePosition(cv::Mat& frame, std::vector<TObject>& boxes)
{
    cv::Rect rect;
    bool Success=false;
    cv::Rect MemRect;
    size_t MemN = 1000000;
    float  MemEdge = 0.0;
    float RatioEdge, RatioRect;
    int tooTall, tooWide;
    float  outsideFocusX;
    float  outsideFocusY;
    unsigned int Wd=frame.cols;
    unsigned int Ht=frame.rows;
    vector<vector<cv::Point>> Cont;           //contours
    vector<cv::Vec4i> Hier;                   //hierarchy
    cv::Mat BWframe, Tframe, Mframe, Eframe, Dframe;

    Trace.Init();       //set the time tag

    for(auto &i : boxes) {
        if((i.obj_id == 0)){       //person
            //a known issue; the whole image is selected as an object -> skip this result
            if((100*i.w>(95*Wd)) || (100*i.h>(95*Ht))) continue;    //stay in the integer domain
            //Create the rectangle
            if((i.w > Wd*Js.PersonMinWidth) && (i.h > Ht*Js.PersonMinHeight) &&    //get some width and height
               ((i.x + i.w) < Wd) && ((i.y + i.h) < Ht)){
                //mark as used
                i.Used = true;
                Trace.Add(frame, i);
            }
        }
        if((i.obj_id == 2)){//||(i.obj_id == 7)){       //car or truck
            //a known issue; the whole image is selected as an object -> skip this result
            if((100*i.w>(95*Wd)) || (100*i.h>(95*Ht))) continue;    //stay in the integer domain
            //Create the rectangle
            if((i.w > Wd*Js.VehicleMinWidth) && (i.h > Ht*Js.VehicleMinHeight) &&    //get some width and height
               ((i.x + i.w) < Wd) && ((i.y + i.h) < Ht)){
                //mark as used
                i.Used = true;

                //get the frame with the vehicle and convert it to gray scale
                cv::cvtColor(frame(cv::Rect(i.x, i.y, i.w, i.h)),BWframe, CV_BGR2GRAY);

                //get morphology
                cv::morphologyEx(BWframe, Mframe, cv::MORPH_BLACKHAT, MorphKernal);

                //get vertical(!) edges -> license plates has a lot of them
                cv::Sobel(Mframe,Eframe,CV_16S,1,0,3);
                cv::convertScaleAbs(Eframe,Eframe);
                cv::threshold(Eframe, Eframe, 64, 255, cv::THRESH_BINARY);

                //convert morhology to binairy image (0 or 255)
                cv::threshold(Mframe, Tframe, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

                //enlarge the white pixels
                cv::dilate(Tframe, Dframe, MorphKernal);

                //get the contours around the white blobs
                findContours(Dframe, Cont, Hier, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
                ///please  note, contours are polygon, a collection of line segments around a 2D-blob, or contour.
                ///we place rectangles around the polygons in order to get simple geographic information
                ///next, search for a rectangle that has the shape of a license plate
                //find the best blob (proper size and ratio, not to far from the border and with a lot of edges)
                for(size_t n=0; n<Cont.size(); n++){
                    rect = cv::boundingRect(Cont[n]);
                    RatioEdge = (float)(100*cv::countNonZero(Eframe(rect)))/(rect.width*rect.height);
                    RatioRect = (float)rect.width / (float)rect.height;
                    tooTall = rect.height > 100;
                    tooWide = rect.width < 70 || rect.width > 400;
                    outsideFocusX = rect.x < 0.10 * Dframe.cols || rect.x > 0.90 * Dframe.cols;
                    outsideFocusY = rect.y < 0.10 * Dframe.rows || rect.y+rect.height > 0.95 * Dframe.rows;
                    if (tooTall || tooWide || outsideFocusX || outsideFocusY || RatioEdge < 7.0f || RatioRect < 0.8 || RatioRect > 5.0) { ; }
                    else {
                        if(MemEdge < RatioEdge){ MemEdge = RatioEdge; MemN = n; MemRect = rect; }
                    }
                }
                if(MemEdge > 3.0 && MemN<Cont.size()){
                    i.PlateFound = true;
                    i.PlateRect  = cv::Rect(MemRect.x+i.x, MemRect.y+i.y, MemRect.width, MemRect.height);
                    i.PlateMedge = MemEdge;      //MemEdge hold the highest edge ratio found in MemRect
                }
                Trace.Add(frame, i);
            }
        }
    }

    Trace.Update();     //check if there are any objects with an old time tag (they have left the scene)

    return Success;


//    Tbegin = std::chrono::steady_clock::now();
//
//                    //detect plates
//                    cv::Mat frame_plate = frame(i.PlateRect);
//                    result_ocr = OcrNet.detect(frame_plate,Js.ThresOCR);
//
//    Tend = std::chrono::steady_clock::now();
//    Msec+= std::chrono::duration_cast<std::chrono::milliseconds> (Tend - Tbegin).count();
//    Mcnt++;


}
//---------------------------------------------------------------------------
void TProcessPipe::DrawObjects(cv::Mat& frame, const std::vector<TObject>& boxes)
{
    char text[256];
    size_t i;
    int x,y,baseLine;

    if(Js.PrintOnCli){
        cout << CamGrb->CamName << endl;
        for(i = 0; i < boxes.size(); i++){
            cout <<boxes[i].prob<<" "<< class_names[boxes[i].obj_id] << endl;
        }
    }

//    cout << Trace.Rbusy.size() <<" - "<< Trace.Rdone.size() << endl;

    if(Js.PrintOnRender || Js.MJPEG_Port>7999){
        //DNN_rect
        cv::rectangle(frame, cv::Point(DnnRect.x, DnnRect.y),
                             cv::Point(DnnRect.x+DnnRect.width, DnnRect.y+DnnRect.height), cv::Scalar(255,255,0),3);
        //Vehicle_rect
        cv::rectangle(frame, cv::Point(Trace.VehicleRect.x, Trace.VehicleRect.y),
                             cv::Point(Trace.VehicleRect.x+Trace.VehicleRect.width, Trace.VehicleRect.y+Trace.VehicleRect.height), cv::Scalar(255,128,0),2);
        //the objects
        for(i = 0; i < boxes.size(); i++) {
            if(boxes[i].Used){
                if(boxes[i].track_id > 0 ){
                    //get a color
                    const unsigned char* Mcol = MarkColor[boxes[i].track_id % 6];
                    cv::Scalar mc(Mcol[0],Mcol[1],Mcol[2]);

                    sprintf(text, " %i ", boxes[i].track_id);

                    baseLine = 0;
                    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseLine);

                    x = boxes[i].x;
                    y = boxes[i].y - label_size.height - baseLine;
                    if (y < 0) y = 0;
                    if (x + label_size.width > frame.cols) x = frame.cols - label_size.width;

                    cv::putText(frame, text, cv::Point(x, y + label_size.height),
                                cv::FONT_HERSHEY_SIMPLEX, 0.8, mc, 2);
                    cv::rectangle(frame, cv::Point(boxes[i].x, boxes[i].y),
                                          cv::Point(boxes[i].x+boxes[i].w, boxes[i].y+boxes[i].h), mc, 2);
                    //the plates
                    if(boxes[i].PlateFound) cv::rectangle(frame,boxes[i].PlateRect,cv::Scalar(0, 255, 0),2);
                }
                else{
                    cv::rectangle(frame, cv::Point(boxes[i].x, boxes[i].y),
                                          cv::Point(boxes[i].x+boxes[i].w, boxes[i].y+boxes[i].h), cv::Scalar(255,0,0),2);
                }
            }
            else{
                cv::rectangle(frame, cv::Point(boxes[i].x, boxes[i].y),
                                      cv::Point(boxes[i].x+boxes[i].w, boxes[i].y+boxes[i].h), cv::Scalar(255,0,0));
                }
         }
    }
}
//----------------------------------------------------------------------------------

