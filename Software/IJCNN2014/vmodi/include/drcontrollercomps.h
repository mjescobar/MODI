#ifndef _DR_CONTCOMPS_H_
#define _DR_CONTCOMPS_H_
#include "modi.h"
#include <vector>
#include <map>
#include <list>

class drMemory
{
public:
    drMemory();
    ~drMemory() {}
    void setValue(std::string key, float value);
    float getValue(std::string key);
    std::vector<pt> corners;
    std::list<int> window;
private:
    std::map<std::string,float> memory;
};

/*
class drSelfController{
		// 	EMPTY!
}
*/

class drControl
{
public:
    drControl();
    ~drControl() {}
    drControl(int threshold);
    void attachX(std::vector<int> *ixvec);
    void attachBeta(std::vector<int> *ibeta);
    void attachMemory(drMemory *imem);
    void linkPos(pt *posi);
    void doControl();
    drMemory *mem;
    std::vector<int> *beta;
    std::vector<int> *xvec;
    pt *current_position;
private:
    int checkCorner(pt position);
    int threshold;
    int prev,curr;
    int radius;
    int end_cond;

};
class drControl2
{
public:
    drControl2();
    ~drControl2() {}
    drControl2(int threshold);
    void attachX(std::vector<int> *ixvec);
    void attachBeta(std::vector<int> *ibeta);
    void attachMemory(drMemory *imem);
    void linkPos(pt *posi);
    void doControl();
    void doControl2();
    drMemory *mem;
    std::vector<int> *beta;
    std::vector<int> *xvec;
    pt *current_position;
private:
    int checkCorner(pt position);
    int checkCorner2(pt position,int skipped);
    int threshold;
    int prev,curr;
    int radius;
    int end_cond;

};
#endif
