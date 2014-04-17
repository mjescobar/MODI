#ifndef _DE_CONT_H_
#define _DE_CONT_H_
#include "modi.h"
#include <cstdlib>
#include <vector>


class decontroller{
public:
	decontroller();
	~decontroller(){}
	decontroller(int IminNoObj, int IminObjLat, int critObjLat, float ialpha);///
	void attachTo(modi *irob);///
	void update();///
	void blockBeta(bool state);///
	void attachBeta(std::vector<int> *vbeta);///
	unsigned char* getActuation();///
	/// public access variables
	std::vector<int> *beta;
	std::vector<int> *xvec;
	modi *robot;
private:
	int cState;
	bool isBblocked;
	int cs, ls, rs;
	unsigned char* control;
	unsigned char prevcontrol[2];
	int minNoObj;
	int minObjLat;
	int critObjLat;
	float alpha;
};

#endif
