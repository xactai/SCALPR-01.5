#ifndef TRACER_H
#define TRACER_H
#include "General.h"
#include "TOCR.h"
#include <opencv2/opencv.hpp>
//----------------------------------------------------------------------------------
class TTracer : public TOCR
{
friend class TProcessPipe;
public:
    TTracer();
    virtual ~TTracer();
    void Init(void);
    void Add(const cv::Mat &frame, TObject& box, const cv::Rect &DnnRect);
    void Update(void);
    void Clean(void);
    std::vector<TRoute> Rdone;
    std::vector<TRoute> Rbusy;
protected:
    cv::Rect    VehicleRect;                    //vehicle must fall inside the rect, otherwise it will not be traced.
private:
    size_t  Tag;
};
//----------------------------------------------------------------------------------
#endif // TRACER_H
