#ifndef TJSON_H
#define TJSON_H

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <fstream>

using json = nlohmann::json;

//----------------------------------------------------------------------------------------
class Tjson
{
friend class ThreadCam;
friend class TProcessPipe;
public:
    Tjson();
    ~Tjson();
    bool LoadFromFile(const std::string FileName);
    bool GetSettings(void);
    void MakeFolders(void);
    //------------------------------------------------------
    //your config.json settings
    int UsedStreams;          //number of used streams (usually nr of cameras)
    int WorkWidth;            //Width of the processed frames (small->fast, large->accurate but slow)
    int WorkHeight;           //Height of the processed frames
    int ThumbWidth;           //Width of the frames shown in the overview (8090 output)
    int ThumbHeight;          //Height of the shown frames
    ///to filter out the background with its cars and tuktuks
    double VehicleMinWidth;   //Minimal width ratio of vehicles to process (no background traffic)
    double VehicleMinHeight;  //Minimal height ratio of vehicles to process (no background traffic)
    double TwoWheelMinWidth;  //Minimal width ratio of two wheelers to process (no background traffic)
    double TwoWheelMinHeight; //Minimal height ratio of two wheelers to process (no background traffic)
    ///persons must also be on the foreground
    double PersonMinWidth;    //Minimal width ratio of persons to process
    double PersonMinHeight;   //Minimal height ratio of persons to process
    ///inspection
    double InspectTime;       //Minimal width ratio of persons to process
    double InspectDistance;   //Minimal height ratio of persons to process
    ///decision weights when a image has a 'better' readable license plate
    ///0 is turned off| >0 working | can be a mix like Wwidth=1.0 Wedge=2.5 Wspeed=12.5
    double Wwidth;            //Width of plate
    double Wedge;             //Edges found in plate (remember H265 compression)
    double Wspeed;            //Average speed

    std::string  Version;     //Version string
    std::string  Gstr;        //Gstreamer string
    std::string  Cstr;        //Car darknet model file name
    std::string  Lstr;        //License darknet model file name
    std::string  Ostr;        //OCR darknet model file name
    bool   PrintOnCli;        //Show license plate on teminal
    bool PrintOnRender;       //show license plate in window
    bool HeuristicsOn;        //Sort character position and remove doubles
    double   ThresCar;        //threshold detection of car model
    double ThresPlate;        //threshold detection of license plate model
    double   ThresOCR;        //threshold detection of ocr model
    std::string FoI_Folder;   //directory with analysed frames
    std::string Car_Folder;   //directory with analysed cars
    std::string Plate_Folder; //directory with analysed plates
    std::string Json_Folder;  //directory with json outputs
    std::string Render_Folder; //directory with ocr results
    int MJPEG_Port;           //output stream (0 = no stream, usually 8090)
    int MJPEG_Width;          //output image width 8090 stream
    int MJPEG_Height;         //output image height 8090 stream
    cv::Rect RoiCrop;         //cropped roi (width==0 and/or height==0 no cropping)
    //------------------------------------------------------
private:
    json j;
    bool Jvalid;
    bool GetSetting(const json& s, const std::string Key, int& Value);
    bool GetSetting(const json& s, const std::string Key, bool& Value);
    bool GetSetting(const json& s, const std::string Key, double& Value);
    bool GetSetting(const json& s, const std::string Key, std::string& Value);
};
//----------------------------------------------------------------------------------------

#endif // TJSON_H
