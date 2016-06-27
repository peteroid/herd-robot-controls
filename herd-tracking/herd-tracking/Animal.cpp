//
//  Animal.cpp
//  herd-tracking
//
//  Created by PETEroid on 4/24/16.
//  Copyright © 2016 How.in.AI. All rights reserved.
//

#include "Animal.hpp"

Animal::Animal() : Object() {
    Animal::role = role_herd;
}

Animal::Animal(string name) : Object(name) {
    Animal::role = role_herd;
    
    if(name=="blue"){
        Animal::defaultMotor = Vec2i(550, 420);
        levelsOutput[0] = defaultMotor;
        levelsOutput[1] = defaultMotor;
        levelsOutput[2] = defaultMotor;
        levelsOutput[3] = defaultMotor;

    } else if(name=="red") {
        Animal::defaultMotor = Vec2i(530, 600);
        levelsOutput[0] = defaultMotor;
        levelsOutput[1] = Vec2i(420, 570);
        levelsOutput[2] = Vec2i(350, 500);
        levelsOutput[3] = Vec2i(0, 480);
        
    } else if(name=="green") {
        Animal::defaultMotor = Vec2i(550, 550);
        levelsOutput[0] = defaultMotor;
        levelsOutput[1] = Vec2i(440, 480);
        levelsOutput[2] = Vec2i(340, 460);
        levelsOutput[3] = Vec2i(0, 400);
        
    } else if(name=="yellow") {
        Animal::defaultMotor = Vec2i(580, 600);
        levelsOutput[0] = defaultMotor;
        levelsOutput[1] = Vec2i(470, 510);
        levelsOutput[2] = Vec2i(400, 440);
        levelsOutput[3] = Vec2i(0, 420);
        
    }
}

int Animal::getDefaultAMotor() {
    return Animal::defaultMotor[0];
}

int Animal::getDefaultBMotor() {
    return Animal::defaultMotor[1];
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
    
    isOutside = abs(x) > ANIMAL_SCREEN_BOUND_X || abs(y) > ANIMAL_SCREEN_BOUND_Y;
    
    clock_t end = clock();
    float duration = 1000.0f * (end - Animal::lastClock) / CLOCKS_PER_SEC;
    Vec2f v = (curPos - lastPos) * (1.0 / duration);
//    cout << norm(v) << endl << ANIMAL_VELOCITY_THRESHOLD << endl;
    if (Animal::_velocities.size() == ANIMAL_VELOCITY_FRAME_SIZE) {
        Animal::velocity -= _velocities.front();
        Animal::_velocities.pop();
    }
    
    Animal::_velocities.push(v);
    Animal::velocity += _velocities.back();
    Animal::lastClock = clock();
}

float Animal::getFlockRotation() {
    return Animal::flockRotation;
};

Vec2f Animal::getSteer(Vec2f desired) {
    float d = norm(desired);
    desired = normalize(desired);
    if (d < ANIMAL_ARRIVE_RANGE) {
        desired *= (d / ANIMAL_ARRIVE_RANGE * ANIMAL_MAXSPEED);
    } else {
        desired *= ANIMAL_MAXSPEED;
    }
    
    Vec2f steer = desired - Animal::velocity;
    return limit(steer);
}

Vec2f Animal::getSeek (Vec2f target) {
    Vec2f desired = target - pointToVec(Object::getPos());
    return getSteer(desired);
}

bool Animal::isNear (Animal target, float dist) {
    float d = distance(Object::getPos(), target.getPos());
    return (d > 0) && (d < dist) && target.isOnScreen;
}

Vec2f Animal::getBoundForce () {
    if (abs(Object::getXPos()) > ANIMAL_SCREEN_BOUND_X) {
        Vec2f desired = Vec2f ((Object::getXPos() > 0? -ANIMAL_MAXSPEED : ANIMAL_MAXSPEED), velocity[1]);
        return getSteer(desired);
    } else if (abs(Object::getYPos()) > ANIMAL_SCREEN_BOUND_Y) {
        Vec2f desired = Vec2f (velocity[0], (Object::getYPos() > 0? -ANIMAL_MAXSPEED : ANIMAL_MAXSPEED));
        return getSteer(desired);
    }
    
    return Vec2f(0, 0);
}

Vec2f Animal::getSeparationForce(vector<Animal> animals, float dist, AnimalRole r) {
    float separationDist = dist;
    Vec2f sum = Vec2f(0, 0);
    int count = 0;
    for (vector<Animal>::iterator it = animals.begin(); it != animals.end(); ++it) {
        float d = distance(Object::getPos(), it->getPos());
        if (isNear(*it, separationDist) && Animal::role == r) {
            Vec2f diff = Object::getPos() - it->getPos();
            sum = sum + (normalize(diff) / d);
            count++;
        }
    }
    
    if (count > 0) {
        sum = sum / count;
        Vec2f steer = (normalize(sum) * ANIMAL_MAXSPEED) - Animal::velocity;
        return limit (steer);
    } else {
        return Vec2f (0, 0);
    }
}

Vec2f Animal::getAlignmentForce(vector<Animal> animals, float dist) {
    float neighbourDist = dist;
    Vec2f sum = Vec2f(0, 0);
    int count = 0;
    for (vector<Animal>::iterator it = animals.begin(); it != animals.end(); ++it) {
        
        if (isNear(*it, neighbourDist)) {
            sum = sum + it->velocity;
            count++;
        }
    }
    
    if (count > 0) {
        sum = sum / count;
        return getSteer(sum);
    } else {
        return Vec2f(0, 0);
    }
}

Vec2f Animal::getCohesionForce(vector<Animal> animals, float dist) {
    float neighbourDist = dist;
    Vec2f sum = Vec2f(0, 0);
    int count = 0;
    for (vector<Animal>::iterator it = animals.begin(); it != animals.end(); ++it) {
        
        if (isNear(*it, neighbourDist)) {
            sum = sum + pointToVec(it->getPos());
            count++;
        }
    }
    
    if (count > 0) {
        sum = sum / count;
        return getSeek(sum);
    } else {
        return Vec2f(0, 0);
    }
}

void Animal::flock (vector<Animal> animals) {
    Vec2f e = getSeparationForce(animals, 1000, role_predator) * 3;
    Vec2f s = getSeparationForce(animals, 500, role_herd);
    Vec2f a = getAlignmentForce(animals, 1000);
    Vec2f c = getCohesionForce(animals, 800) * 0.5;
    Vec2f b = getBoundForce();
    
    Animal::finalForce = s;
    Animal::finalForce = Animal::finalForce + a;
    Animal::finalForce = Animal::finalForce + c;
    Animal::finalForce = Animal::finalForce + b;
    Animal::finalForce = Animal::finalForce + e;
    
    flockRotation = getDirectionFrom(Animal::finalForce, true);
    float currentRotation = getDirectionFrom(velocity, true);
    
    float diff = currentRotation - flockRotation;
    int newTurnLevel = turnLevel;
    
    float absDiff = diff;
    while (absDiff > 360) {
        absDiff -= 360;
    }
    while (absDiff < 0) {
        absDiff += 360;
    }
    
    bool isPreviousClockwise = isClockwise;
    isClockwise = absDiff > 180;
    if (isClockwise) {
        // prepare for the next checking
        absDiff = 360 - absDiff;
    }
    
    if (absDiff > 90) {
        // lv3
        newTurnLevel = 3;
    } else if (absDiff > 45) {
        // lv2
        newTurnLevel = 2;
    } else if (absDiff > 10) {
        // lv1
        newTurnLevel = 1;
    } else {
        newTurnLevel = 0;
    }
    
    turnLevel = newTurnLevel;
    
    cout << getType() << s;
    cout << ':' << a;
    cout << ':' << c;
    cout << ':' << b;
    cout << ':' << e;
//    cout << ':' << Animal::finalForce;
    cout << ':' << flockRotation;
    cout << ':' << turnLevel;
    cout << endl << endl;
}

float Animal::getDirectionFrom (Vec2f from, bool isPointing) {
    double shiftAngle = 360 - (atan2 (from[1], from[0]) * 180 / M_PI - 90) +
    ANIMAL_NORTH_ANGLE + (isPointing ? 0 : 180);
    while (shiftAngle > 360) {
        shiftAngle -= 360;
    }
    
    while (shiftAngle < 0) {
        shiftAngle += 360;
    }
    
    return shiftAngle;
}
