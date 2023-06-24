## Multithreaded Simultaneous License Plate location. 

### Tracking and recognize license plates.
👉 please read the explanation at [YoloV4tinyMulti3](https://github.com/xactai/SCALPR-01.5/tree/main/YoloV4tinyMulti3), [YoloMultiDusty](https://github.com/xactai/SCALPR-01.5/tree/main/YoloMultiDusty) and [YoloMultiDusty2](https://github.com/xactai/SCALPR-01.5/tree/main/YoloMultiDusty2) first. This repo is the sequel.<br><br>
The first study, YoloMultiDusty, showed that multithreading is the best solution for connecting multiple inputs to a Jeston board.<br>
The second study, YoloMultiDusty2, adds json configuration files and deep learning preparations.<br><br>
The third study, YoloV4tinyMulti3, uses the YoloV4-tiny model and starts detecting license plates in CCTV images.<br>
This study is all about tracking objects in time and space.<br>

------------

## How it works.
![image](https://github.com/xactai/SCALPR-01.5/assets/44409029/3aff2b06-8637-46c8-9432-f672bab9d2f5)

#### DNN_Rect
In the first phase, all objects in the `DNN_Rect` are detected by the YoloV4-tiny model.<br>
Most models, such as YoloV4, work best with square images. `DNN_Rect` defines an area of interest that is more or less square.<br>
In the screenshot above, the `DNN_Rect` is marked with a light blue rectangle. Square is not mandatory; it only improves the recognition rate.<br>
Because vehicles avoid the right wall, we can crop out a large part of the image and we're left with a square-like region.<br>
All detected object in `DNN_Rect` are marked with a thin blue rectangle (chair, monitor, guard outside).
In the `config.json` `DNN_Rect` is defined in pixels.

#### Foreground
The next phase selects only persons, two and four-wheelers, in the foreground.<br>
YoloV4 recognize a lot of objects. Most of them are in the street in the background; pedestrians, traffic like cars, tuk-tuks, etc.<br>
The most simple and effective way to filter those objects out is to look at their size and class (person, car, truck, tv-monitor, chair).<br>
In `config.json`, parameters `VEHICLE_MIN_WIDTH` and `VEHICLE_MIN_HEIGHT` define the size of the vehicles in the foreground.<br>
The same applies to the parameters `PERSON_MIN_WIDTH` and `PERSON_MIN_HEIGHT`. People smaller than the given numbers are considered in the background, like the guard in its blue uniform.<br>
Taller persons, like the man next to the car, are located in the foreground.
The numbers in the 'config.json' are relative to the frame resolution. For instance, an IMOU camera with 1920x1080 frames and a `VEHICLE_MIN_WIDTH` of 0.18, results in 0.18 x 1920 = 346. Only vehicles larger than 346 pixels are processed further. Every other vehicle is ignored.<br>
Using relative numbers is convenient because you don't have to change them every time the camera's resolution differs. After all, the dimensions remain the same.

#### Tracking
Once an object is situated in the foreground it will be tracked. A tracking routine tries to follow the object's movements.<br>
Darknet has a simple tracking routine incorporated into its framework. It is based on [Optical Flow](https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html).<br>
Objects who are tracked are marked with a thick rectangle. The tracking number is placed at the top right corner. Note, every class has its own tracking numbers. That's why the person and the car have the same tracking number 2.<br>
To highlight possible tracking errors, the color of the rectangle depends on the tracking number.

#### Route
At this stage, the static frame-by-frame domain transitions into the tractory domain. In other words, only movements are observed.<br> 
Once an object has a sequence number, a structure is created and added to the Rbusy array (Route Busy).<br>
Note, in addition to vehicles, people are tracked once they are detected also.<br>
The structure contains all essential information about the object.<br>
```cpp
// bbox_t
    unsigned int x, y, w, h;                        //(x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;                                     //confidence - probability that the object was found correctly
    unsigned int obj_id;                            //class of object - from range [0, classes-1]
    unsigned int track_id;                          //tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter;                    //counter of frames on which the object was detected
    float x_3d, y_3d, z_3d;                         //center of object (in Meters) if ZED 3D Camera is used
// TObject
    bool     Used {false};                          //this object (vehicle or person) is further investigated.
    bool     Done {false};                          //this object has crossed the vehicle rect border and is considered processed.
    bool     PlateFound {false};                    //license plate found
    cv::Rect PlateRect;                             //location of the plate rectangle
    float    PlateMedge;                            //ratio of edges
    float    PlateFuzzy {0.0};                      //output weighted calcus for better plate
    cv::Mat  Plate;                                 //image of the plate
// TRoute    
    size_t  FirstFrame;                             //used for stitching two routes to one
    size_t  LastFrame;                              //when not updated, you know the object has left the scene
    std::chrono::steady_clock::time_point Tstart;   //time stamp first seen
    std::chrono::steady_clock::time_point Tend;     //time stamp last seen
    int Xc {0};                                     //ROI center (x,y)
    int Yc {0};                                     //ROI center (x,y)
    int Xt, Yt;                                     //previous center (x,y)
    float Velocity;                                 //pixels/100mSec (0.1Sec to increase readability)
    TMoveAver<float> Speed;                         //average speed over the last 5 observations
    float PlateSpeed;                               //average speed when snapshot of license was taken
    float PlateEdge;                                //edge ratio in plate
    float PlateRatio {0.0};                         //plate width/height
    std::string PlateOCR;                           //OCR string with best result of the plate (if found)
```
It's a long list because C++ inheritance is used here.<br>
TRoute inherits all elements from TObject, which in turn has already inherited the structure bbox_t from Darknet.<br>

------------

## Dependencies.
The dependencies are the same as in previous versions.

------------

## Config.json.
All required settings are listed in the `config.json` file. Without this file, the app will not work.
```json
{
    "VERSION": "1.1.0",

    "STREAMS_NR":1,

    "STREAM_1": {
        "CAM_NAME": "Cam 1",
        "MJPEG_PORT": 8091,
        "VIDEO_INPUT": "video",
        "VIDEO_INPUTS_PARAMS": {
            "image": "./images/car.jpg",
            "folder": "./inputs/images",
            "video": "./Cars.mp4",
            "usbcam": "v4l2src device=/dev/video0 ! video/x-raw(memory:NVMM),width=640, height=360, framerate=30/1 ! videoconvert ! appsink",
            "CSI1": "nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=640, height=480, framerate=15/1, format=NV12 ! nvvidconv ! video/x-raw, format=BGRx, width=640, height=480 ! videoconvert ! video/x-raw, format=BGR ! appsink",
            "CSI2": "nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM),width=640, height=480, framerate=15/1, format=NV12 ! nvvidconv ! video/x-raw, format=BGRx, width=640, height=480 ! videoconvert ! video/x-raw, format=BGR ! appsink",
            "CCTV": "rtsp://192.168.178.20:8554/test/",
            "IMOU": "rtsp://admin:L230BF4A@192.168.178.40:554/cam/realmonitor?channel=1&subtype=0",
            "remote_hls_gstreamer": "souphttpsrc location=http://YOUR_HLSSTREAM_URL_HERE.m3u8 ! hlsdemux ! decodebin ! videoconvert ! videoscale ! appsink"
        },
        "DNN_Rect": {
            "x_offset": 100,
            "y_offset": 0,
            "width":  1280,
            "height": 1080
        },
        "VEHICLE_Rect": {
            "left": 0.234,
            "top": 0.0,
            "right":  0.0,
            "bottom": 0.185
        }
    },

    "STREAM_2": {
        "CAM_NAME": "Cam 2",
        "MJPEG_PORT": 8092,
        "VIDEO_INPUT": "CCTV",
        "VIDEO_INPUTS_PARAMS": {
            "CCTV": "rtsp://192.168.178.26:8554/test/"
        },
        "DNN_Rect": {
            "x_offset": 0,
            "y_offset": 0,
            "width":  640,
            "height": 480
        },
        "VEHICLE_Rect": {
            "left": 0.234,
            "top": 0.0,
            "right":  0.0,
            "bottom": 0.185
        }
    },

    "WORK_WIDTH": 1920,
    "WORK_HEIGHT": 1080,
    "THUMB_WIDTH": 1280,
    "THUMB_HEIGHT": 720,
    "VEHICLE_MIN_WIDTH" : 0.18,
    "VEHICLE_MIN_HEIGHT" : 0.15,

    "PERSON_MIN_WIDTH" : 0.04,
    "PERSON_MIN_HEIGHT" : 0.07,
   
    "WEIGHT_WIDTH": 0.25,
    "WEIGHT_EDGE": 4.0,
    "WEIGHT_SPEED": 1.5,
  
   
```
Most of the entries are well-known from the previous versions. Check out the READMEs for more information.<br>
#### DNN_Rect
Now every CCTV has its unique DNN_Rect. It defines the region that will use the DNN model.
#### MJPEG_PORT
Also new is the entry MJPEG_PORT per CCTV. Now, each processed image can be viewed individually.
#### VEHICLE_MIN_WIDTH, VEHICLE_MIN_HEIGHT
Only a vehicle entering the parking lot is of interest. Traffic on the road in the background should be ignored.<br>
The parameters determine the minimum dimensions of a vehicle. The numbers are ratios of the WORK_WIDTH and WORK_HEIGHT.<br>

