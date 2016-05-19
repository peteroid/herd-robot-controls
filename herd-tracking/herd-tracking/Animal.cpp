//
//  Animal.cpp
//  herd-tracking
//
//  Created by PETEroid on 4/24/16.
//  Copyright © 2016 How.in.AI. All rights reserved.
//

#include "Animal.hpp"

#define ANIMAL_VELOCITY_THRESHOLD 0.1

Animal::Animal() : Object() {
    
}

Animal::Animal(string name) : Object(name) {
    
}

Vec2f Animal::getVelocity() {
    return Animal::velocity;
}

void Animal::setVelocity(Vec2f v) {
    Animal::velocity = v;
}

void Animal::setPos(float x, float y) {
    Animal::lastPos = Object::curPos;
    Object::setPos(x, y);
    
    clock_t end = clock();
    float duration = 1000.0f * (end - Animal::lastClock) / CLOCKS_PER_SEC;
    Vec2f v = (curPos - lastPos) * (1.0 / duration);
//    cout << norm(v) << endl << ANIMAL_VELOCITY_THRESHOLD << endl;
    if (norm(v) > ANIMAL_VELOCITY_THRESHOLD)
        Animal::setVelocity(v);
    else
        Animal::setVelocity(Vec2f());
    Animal::lastClock = clock();
}