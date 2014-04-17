#ifndef _DR_CONT_H_
#define _DR_CONT_H_
#include "drcontrollercomps.h"
#include <cstdlib>


class drcontroller{
public:
	drcontroller();
	~drcontroller(){}
	drcontroller(int ithreshold,int speed, pt *posi);
	void update();
	void attachX(std::vector<int> *ixvec);
    void linkPos(pt *posi);
    std::vector<int> *beta;
	std::vector<int> *xvec;
private:
    pt *current_position;
	drMemory *mem;
	drControl *control;
	int threshold;
};
#endif
