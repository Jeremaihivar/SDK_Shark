#include "hchead.h"
#include <chrono>


bool nodeComparator(const tsNodeInfo& s1, const tsNodeInfo& s2)
{
	return s1.angle_q6_checkbit < s2.angle_q6_checkbit;
}

bool newComparator(const tsPointCloud& s1, const tsPointCloud& s2)
{
	return s1.dAngle > s2.dAngle;
}


HCHead::HCHead()
{

}


UINT64 HCHead::getCurrentTimestampUs()
{
    auto ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return ts;
}


void HCHead::eraseBuff(std::vector<UCHAR>& lstG,int iLen)
{
	if (lstG.size() >= iLen)
		lstG.erase(lstG.begin(), lstG.begin() + iLen);
	else
		lstG.clear();

    {
         std::vector<UCHAR> tmp = lstG;
         lstG.swap(tmp);
    }
}


void HCHead::eraseRangeData(LstPointCloud& lstG,int iLen)
{
	if (lstG.size() >= iLen)
		lstG.erase(lstG.begin(), lstG.begin() + iLen);
	else
		lstG.clear();

    {
         LstPointCloud tmp = lstG;
         lstG.swap(tmp);
    }
}
