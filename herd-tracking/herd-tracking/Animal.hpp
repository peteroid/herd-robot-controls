//
//  Animal.hpp
//  herd-tracking
//
//  Created by PETEroid on 4/24/16.
//  Copyright © 2016 How.in.AI. All rights reserved.
//

#include "Object.h"
#include <ctime>

#ifndef Animal_hpp
#define Animal_hpp

class Animal : public Object {
    Vec2f velocity;
    clock_t lastClock;
    Point2f lastPos;
    
public:
    Animal();
    Animal(string name);
    Vec2f getVelocity();
    void setVelocity(Vec2f v);
    void setPos(float x, float y);
};

#endif /* Animal_hpp */
