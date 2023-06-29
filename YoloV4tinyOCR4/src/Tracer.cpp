#include "Tracer.h"
#include "Tjson.h"

extern Tjson    Js;
//----------------------------------------------------------------------------------
TTracer::TTracer()
{
    Tag=0;
}
//----------------------------------------------------------------------------------
TTracer::~TTracer()
{
}
//----------------------------------------------------------------------------------
void TTracer::Init(void)
{
    Tag++;
}
//----------------------------------------------------------------------------------
void TTracer::Add(const cv::Mat &frame, TObject& box, const cv::Rect &DnnRect)
{
    size_t i, n, p;
    bool Found=false;

    //check if the car is inside the VEHICLE_Rect
    if(box.obj_id==2){
        if((box.x       >= (unsigned int)VehicleRect.x                   ) &&
           (box.y       >= (unsigned int)VehicleRect.y                   ) &&
           (box.x+box.w <= (unsigned int)VehicleRect.x+VehicleRect.width ) &&
           (box.y+box.h <= (unsigned int)VehicleRect.y+VehicleRect.height)){;}
        else{
            box.Used = false;
        }
    }
    else{
        //check if the two-wheeler is inside the DNN_rect
        if((box.obj_id==1) || (box.obj_id==3) ){
            if((box.x       >= (unsigned int)DnnRect.x                   ) &&
               (box.y       >= (unsigned int)DnnRect.y                   ) &&
               (box.x+box.w <= (unsigned int)DnnRect.x+DnnRect.width ) &&
               (box.y+box.h <= (unsigned int)DnnRect.y+DnnRect.height)){;}
            else{
                box.Used = false;
            }
        }
    }

    //check if it is new object
    for(i=0;i<Rbusy.size();i++){
        if((Rbusy[i].obj_id == box.obj_id) && (Rbusy[i].track_id == box.track_id)){ n=i; Found=true; break; }
    }

    if(!Found){
        //new object or an object already done

        ///due to imperfection of DarkNets track algorithm, the object may already be seen before and stored in Rdone

        //so check Rdone first
        for(i=0;i<Rdone.size();i++){
            //               same ID
            if(Rdone[i].obj_id == box.obj_id){
                //               same track             or  almost immediately after the previous
                if((Rdone[i].track_id  == box.track_id) || Rdone[i].LastFrame >= Tag-6 ){ n=i; Found=true; break; } //do not re-enter vehicles considered processed.
            }
        }
        if(Found){
            if(!Rdone[n].Done){     //do not re-enter vehicles considered processed (they have crossed the border)
                //it was seen before, get it back to Rbusy
                p=Rbusy.size();                 //p will be the position in the vector
                Rbusy.push_back(Rdone[n]);
                Rbusy[p].Update(box, frame, Tag);
                //remove the object from Rdone
                Rdone.erase(Rdone.begin() + n);
            }
        }
        else{
            //now we are sure its a new object
            TRoute Rt;
            Rt.Update(box, frame, Tag);
            Rt.FirstFrame = Tag;
            Rt.Tstart     = Rt.Tend;
            Rbusy.push_back(Rt);
        }
    }
    else{
        if(!Rbusy[n].Done){     //do not re-enter vehicles considered processed (they have crossed the border)
            //known object
            Rbusy[n].Update(box, frame, Tag);
            if(!box.Used){
                Rbusy[n].Done = true;
                ///vehicle has crossed the border.
                // it will not re-enter the Busy pipeline, so see if you can read the license plate.
                OCR(Rbusy[n]);
                //it is possible that objects are found in the Done list with the flag Rdone[].Done = false
                //see if is this the case with the previous entries. If so, OCR + Rdone[].Done = true.
                for(size_t k=0; k<Rdone.size(); k++){
                    if(!Rdone[k].Done){
                        OCR(Rdone[k]);
                        Rdone[k].Done=true;
                    }
                }
            }
        }
    }
}
//----------------------------------------------------------------------------------
void TTracer::Update(void)
{
    float Tcontact;

    //see which object isn't been updated
    for(size_t i=0;i<Rbusy.size();i++){
        if(Rbusy[i].LastFrame != Tag){
            //object is not in scene anymore
            if((Rbusy[i].obj_id != 0) && (Rbusy[i].Tstart!=Rbusy[i].Tend)){
                //get inspection info
                if((Rbusy[i].Person_ID >=0) && (Rbusy[i].obj_id == 2)){
                    Tcontact = std::chrono::duration_cast<std::chrono::milliseconds>(Rbusy[i].TLinp-Rbusy[i].TFinp).count();
                    Rbusy[i].Inspected = (Tcontact >= Js.InspectTime);
//                    std::cout << "Inspected : "<< Rbusy[i].Inspected <<"  Tcontact : " << Tcontact << std::endl;
                }
                //move it to Rdone (if not a person, or a one time glitch (Tstart = Tend)
                Rdone.push_back(Rbusy[i]);
            }
            //remove the object from Rbusy
            Rbusy.erase(Rbusy.begin() + i);
            i--;
        }
    }
}
//----------------------------------------------------------------------------------
void TTracer::Clean(void)
{
    //see which object has impossible short times
    for(size_t i=0;i<Rdone.size();i++){
        if(((Rdone[i].LastFrame-Rdone[i].FirstFrame)<3) || (Rdone[i].PlateEdge<0)){
            //object has a too short live span or has negative edges, remove
            Rdone.erase(Rdone.begin() + i);
            i--;
        }
    }
}
//----------------------------------------------------------------------------------
