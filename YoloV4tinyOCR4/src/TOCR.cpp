#include "TOCR.h"
#include "Tjson.h"
#include "Regression.h"
#include "yolo_v2_class.hpp"	        // imported functions from .so

//----------------------------------------------------------------------------------------
using namespace std;
//----------------------------------------------------------------------------------------
extern Tjson    Js;
extern Detector OcrNet;
extern vector<string> OCRnames;

extern int Msec;
extern int Mcnt;
//----------------------------------------------------------------------------------
TOCR::TOCR()
{
    //ctor
}
//----------------------------------------------------------------------------------
TOCR::~TOCR()
{
    //dtor
}
//----------------------------------------------------------------------------------------
//note, SortSingleLine can erase elements from cur_bbox_vec
//that way shorten the length of the vector. Hence size_t& bnd
void TOCR::SortSingleLine(vector<bbox_t>& cur_bbox_vec, float ch_wd, float ch_ht, size_t StartPos, size_t& StopPos)
{
    size_t i, j;
    bbox_t tmp_box;
    int d, i1, i2;

    if((StopPos-StartPos)<=1) return;

    //sort by x position
    for(i=StartPos; i<StopPos; i++){
        for(j=i+1; j<StopPos; j++){
            if(cur_bbox_vec[j].x<cur_bbox_vec[i].x){
                //swap
                tmp_box=cur_bbox_vec[j];
                cur_bbox_vec[j]=cur_bbox_vec[i];
                cur_bbox_vec[i]=tmp_box;
            }
        }
    }

    //get the distance between each char, too close? select the highest prob.
    for(i=StartPos; i<StopPos-1; i++){
        i1=cur_bbox_vec[i].x; i2=cur_bbox_vec[i+1].x;
        d=(i2-i1)*2;            //d<0? two lines and jumping from the top to the bottom line.
        if(d>=0 && d<ch_wd){
            if(cur_bbox_vec[i+1].prob < cur_bbox_vec[i].prob) cur_bbox_vec.erase(cur_bbox_vec.begin()+i+1);
            else                                              cur_bbox_vec.erase(cur_bbox_vec.begin()+i);
            StopPos--;  i--;    //one element less in the array, due to the erase
        }
    }
}
//----------------------------------------------------------------------------------------
void TOCR::SortPlate(vector<bbox_t>& cur_bbox_vec)
{
    size_t i, j, n, bnd;
    size_t len=cur_bbox_vec.size();
    bbox_t tmp_box;
    float ch_wd, ch_ht;
    double A, B, R;
    TLinRegression LinReg;

    if(len < 2) return;         //no need to investigate 1 character

    //get average width and height of the characters
    for(ch_ht=ch_wd=0.0, i=0; i<len; i++){
        ch_wd+=cur_bbox_vec[i].w;
        ch_ht+=cur_bbox_vec[i].h;
    }
    ch_wd/=len; ch_ht/=len;

    if(len > 4){
        //get linear regression through all (x,y)
        for(i=0; i<len; i++){
            LinReg.Add(cur_bbox_vec[i].x, cur_bbox_vec[i].y);
        }
        LinReg.Execute(A,B,R);
        //now you can do a warp perspective if the skew is too large.
        //in that case, you have to run the ocr detection again.
        //here we see how well a single line fits all the characters.
        //if the standard deviation is high, you have one line of text
        //if the R is low, you have a two line number plate.

    //    cout << "A = " << A << "  B = " << B << "  R = " << R << endl;
    }
    else{
        R=1.0;  // with 4 or less characters, assume we got always one line.
    }

    if( R < 0.25 ){
        //two lines -> sort on y first
        for(i=0; i<len; i++){
            for(j=i+1; j<len; j++){
                if(cur_bbox_vec[j].y<cur_bbox_vec[i].y){
                    //swap
                    tmp_box=cur_bbox_vec[j];
                    cur_bbox_vec[j]=cur_bbox_vec[i];
                    cur_bbox_vec[i]=tmp_box;
                }
            }
        }

        //get the boundary between first and second line.
        for(n=0, i=0; i<len-1; i++){
            j=cur_bbox_vec[i+1].y-cur_bbox_vec[i].y;
            if(j>n){ n=j; bnd=i+1; }
        }
        //sort the first line 0 < bnd
        SortSingleLine(cur_bbox_vec, ch_wd, ch_ht, 0, bnd);

        len=cur_bbox_vec.size();        //SortSingleLine can shorten the length of the vector
        //sort the second line bnd < len
        SortSingleLine(cur_bbox_vec, ch_wd, ch_ht, bnd, len);
    }
    else{
        //one line -> sort by x position
        SortSingleLine(cur_bbox_vec, ch_wd, ch_ht, 0, len);
    }
}
//----------------------------------------------------------------------------------------
void TOCR::OCR(TRoute& route)
{
    size_t i;
    char text[32];
    vector<bbox_t> result_ocr;

    std::chrono::steady_clock::time_point Tbegin, Tend;

    Tbegin = std::chrono::steady_clock::now();

    route.PlateOCR=" ?? "; //default, in case no license is recognized.

    if(route.PlateFound){
        //detect plates
        result_ocr = OcrNet.detect(route.Plate,Js.ThresOCR);

        if(result_ocr.size()>0){
            //heuristics
            if(Js.HeuristicsOn){
                SortPlate(result_ocr);
            }
            //transfer to std::string
            for(i=0;(i<result_ocr.size() && i<32);i++){
                text[i]=OCRnames[result_ocr[i].obj_id][0];
            }
            text[i]=0; //closing (0=endl);
            route.PlateOCR=text;
        }
    }

    cout << " -----------------> OCR : " << route.PlateOCR << endl;

    Tend = std::chrono::steady_clock::now();
    Msec+= std::chrono::duration_cast<std::chrono::milliseconds> (Tend - Tbegin).count();
    Mcnt++;
}
//----------------------------------------------------------------------------------
