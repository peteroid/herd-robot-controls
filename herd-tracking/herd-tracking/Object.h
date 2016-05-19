#pragma once
#include <string>
#include <cv.h>
#include <highgui.h>
#include <ctime>

using namespace std;
using namespace cv;

class Object
{
public:
	Object();
	~Object(void);

	Object(string name);

	float getXPos();
	void setXPos(float x);

	float getYPos();
	void setYPos(float y);
    
    Point2f getPos();
    virtual void setPos(float x, float y);
    
	Scalar getHSVmin();
	Scalar getHSVmax();

	void setHSVmin(Scalar min);
	void setHSVmax(Scalar max);

	string getType(){return type;}
	void setType(string t){type = t;}

	Scalar getColor(){
		return Color;
	}
	void setColor(Scalar c){

		Color = c;
	}
    
    bool isOnScreen;
    
protected:
    Point2f curPos;
	string type;
	Scalar HSVmin, HSVmax;
	Scalar Color;
};
