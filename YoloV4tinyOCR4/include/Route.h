#ifndef ROUTE_H_INCLUDED
#define ROUTE_H_INCLUDED

#include "yolo_v2_class.hpp"
#include "Statistic.h"
#include "Tjson.h"

extern Tjson    Js;
//---------------------------------------------------------------------------
struct TObject : public bbox_t
{
    bool     Used {false};              //this object (vehicle or person) is further investigated.
    bool     Done {false};              //this object has crossed the vehicle rect border and is considered processed.
    bool     PlateFound {false};        //license plate found
    cv::Rect PlateRect;                 //location of the plate
    float    PlateMedge;                //ratio of edges
    float    PlateFuzzy {0.0};          //output weighted calcus better plate
    cv::Mat  Plate;                     //image of the plate

    inline void operator=(const bbox_t T1){
        x=T1.x;  y=T1.y; w=T1.w; h=T1.h; prob=T1.prob; obj_id=T1.obj_id;
        track_id=T1.track_id; frames_counter=T1.frames_counter;
    }
    inline void operator=(const TObject T1){
        x=T1.x;  y=T1.y; w=T1.w; h=T1.h; prob=T1.prob; obj_id=T1.obj_id;
        track_id=T1.track_id; frames_counter=T1.frames_counter;
        Used=T1.Used; Done=T1.Done; PlateFound=T1.PlateFound;
        PlateRect=T1.PlateRect; PlateMedge=T1.PlateMedge;
        PlateFuzzy=T1.PlateFuzzy; Plate=T1.Plate.clone();
    }
};
//---------------------------------------------------------------------------
struct TRoute : public TObject
{
    size_t  FirstFrame;                             //used for stitching two routes to one
    size_t  LastFrame;                              //when not updated, you know the object has left the scene
    std::chrono::steady_clock::time_point Tstart;   //time stamp first seen
    std::chrono::steady_clock::time_point Tend;     //time stamp last seen
    int Xc {0};                                     //ROI center (x,y)
    int Yc {0};                                     //ROI center (x,y)
    int Xt, Yt;                                     //previous center (x,y)
    float Velocity;                                 //pixels/100mSec (0.1Sec to increase readability)
    TMoveAver<float> Speed;                         //average speed over the last 5 observations
    float PlateSpeed;                               //average speed when taken snapshot of license
    float PlateEdge;                                //edge ratio in plate
    float PlateRatio {0.0};                         //plate width/height
    std::string PlateOCR;                           //OCR best result of the plate (if found)

    inline void operator=(const TRoute R1){
        PlateFound = R1.PlateFound;
        PlateRect  = R1.PlateRect;
        obj_id     = R1.obj_id;
        track_id   = R1.track_id;
        Used       = R1.Used;
        Done       = R1.Done;
        x          = R1.x;
        y          = R1.y;
        w          = R1.w;
        h          = R1.h;
        FirstFrame = R1.FirstFrame;
        LastFrame  = R1.LastFrame;
        Tstart     = R1.Tstart;
        Tend       = R1.Tend;
        PlateSpeed = R1.PlateSpeed;
        PlateEdge  = R1.PlateEdge;
        PlateRatio = R1.PlateRatio;
        PlateOCR   = R1.PlateOCR;
        Plate      = R1.Plate.clone();
    }

    inline void Update(const TObject& box, const cv::Mat &frame, const size_t FrameCnt){
        float Rwidth, Redge, Rspeed, Aspeed, Wfuzzy;

        obj_id    = box.obj_id;
        track_id  = box.track_id;
        Used      = box.Used;
        Done      = box.Done;
        x         = box.x;
        y         = box.y;
        w         = box.w;
        h         = box.h;
        LastFrame = FrameCnt;
        Xt        = Xc;
        Yt        = Yc;
        Xc        = box.x + (box.w/2);
        Yc        = box.y + (box.h/2);
        Tend      = std::chrono::steady_clock::now();
        //calculate displacement
        float Distance    = sqrt(((Xt-Xc)*(Xt-Xc)) + ((Yt-Yc)*(Yt-Yc)));
        //get time (0.1 Sec) elapse
        float Telapse     = (std::chrono::duration_cast<std::chrono::milliseconds>(Tend-Tstart).count())/100.0;
        //calculate velocity
        if(Telapse>0.0){
            Velocity = Distance/Telapse;
            //due to dancing object boxes it is possible do not have a stable center point.
            //it can generate huge velocities, which must be ignored.
            if(Velocity<100.0) Speed.Add(Velocity);
        }
        else{
            Velocity = 0.0;
        }
        //do we need to update the plate?
        if(box.PlateFound){
            if(!PlateFound){
                //first time plate found
                PlateFound = box.PlateFound;
                PlateRect  = box.PlateRect;
                PlateSpeed = Speed.Aver();
                PlateEdge  = box.PlateMedge;
                PlateRatio = (float)box.PlateRect.width / (float)box.PlateRect.height;
                Plate  = frame(PlateRect).clone();

//            std::string St="Plate"+std::to_string(FrameCnt);
//            St+=".png";
//            cv::imwrite(St,Plate);

            }
            else{

    cv::imshow("Plate Else",Plate);
    cv::waitKey(5);
                //look if its a better plate
                Rwidth  = (PlateRect.width>0)? (float)box.PlateRect.width * Js.Wwidth : 0.0;
                Redge = (PlateEdge>0)? box.PlateMedge * Js.Wedge : 0.0;

                Aspeed=Speed.Aver(); //speed (lower is better)
                if(Aspeed<=0.0) Aspeed=0.0001; //if zero, give it small bias, otherwise the calcus fails
                Rspeed  = Js.Wspeed/Aspeed;
                //add
                Wfuzzy = Rwidth + Redge + Rspeed;

//std::cout << "Width " << box.PlateRect.width << "  Edge " << box.PlateMedge << "  Speed " << Aspeed << "  Fuzzy " << Wfuzzy << std::endl;

                //look if its a better plate
                if(PlateFuzzy < Wfuzzy){
                    PlateFuzzy = Wfuzzy;
                    PlateRect  = box.PlateRect;
                    PlateSpeed = Speed.Aver();
                    PlateEdge  = box.PlateMedge;
                    PlateRatio = (float)box.PlateRect.width / (float)box.PlateRect.height;
                    Plate  = frame(PlateRect).clone();

std::cout << "update better " << FrameCnt << "  PlateFuzzy " << PlateFuzzy << std::endl;

//            std::string St="Plate"+std::to_string(FrameCnt);
//            St+=".png";
//            cv::imwrite(St,Plate);

                }
            }
        }
    }
};
//----------------------------------------------------------------------------------------
#endif // ROUTE_H_INCLUDED
