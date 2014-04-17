#ifndef _DR_CONT2_H_
#define _DR_CONT2_H_
#include "drcontrollercomps.h"
#include <cstdlib>


class drcontroller2{
public:
	drcontroller2();
	~drcontroller2(){}
	drcontroller2(int ithreshold,int speed, pt *posi);
	void update();
	void attachX(std::vector<int> *ixvec);
    void linkPos(pt *posi);
    std::vector<int> *beta;
	std::vector<int> *xvec;
private:
    pt *current_position;
	drMemory *mem;
	drControl2 *control;
	int threshold;
};

#endif
