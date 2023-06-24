#ifndef TOCR_H
#define TOCR_H
#include "General.h"
//----------------------------------------------------------------------------------
class TOCR
{
public:
    TOCR();
    virtual ~TOCR();
    void OCR(TRoute& route);

protected:

private:
    void SortPlate(std::vector<bbox_t>& cur_bbox_vec);
    void SortSingleLine(std::vector<bbox_t>& cur_bbox_vec, float ch_wd, float ch_ht, size_t StartPos, size_t& StopPos);
};
//----------------------------------------------------------------------------------
#endif // TOCR_H
