//
//  Animal.hpp
//  herd-tracking
//
//  Created by PETEroid on 4/24/16.
//  Copyright © 2016 How.in.AI. All rights reserved.
//

#include "Object.h"
#include <ctime>
#include <queue>
#include <math.h>

#ifndef Animal_hpp
#define Animal_hpp

#define ANIMAL_VELOCITY_THRESHOLD 0
#define ANIMAL_VELOCITY_FRAME_SIZE 5

#define ANIMAL_MAXSPEED 3
#define ANIMAL_MAXFORCE 5
#define ANIMAL_ARRIVE_RANGE 50
#define ANIMAL_NORTH_ANGLE 20
#define ANIMAL_SCREEN_BOUND_X 300
#define ANIMAL_SCREEN_BOUND_Y 250
#define ANIMAL_NUM_OF_LEVELS 4

enum AnimalRole {role_herd, role_predator};

class Animal : public Object {
    queue<Vec2f> _velocities;
    Vec2f velocity;
    clock_t lastClock;
    Point2f lastPos;
    float flockRotation;
    Vec2i defaultMotor;
    
public:
    Animal();
    Animal(string name);
    
    Vec2i levelsOutput[ANIMAL_NUM_OF_LEVELS];
    
    int turnLevel;
    bool isOutside;
    bool isClockwise;
    Vec2f finalForce;
    AnimalRole role;
    
    Vec2f getVelocity();
    void setVelocity(Vec2f v);
    void setPos(float x, float y);
    float getFlockRotation();
    int getDefaultAMotor();
    int getDefaultBMotor();
    
    Vec2f getSteer (Vec2f desired);
    Vec2f getSeek (Vec2f target);
    Vec2f getBoundForce ();
    bool isNear (Animal target, float dist);
    
    Vec2f getSeparationForce(vector<Animal> animals, float dist, AnimalRole r);
    Vec2f getAlignmentForce(vector<Animal> animals, float dist);
    Vec2f getCohesionForce(vector<Animal> animals, float dist);
    void flock (vector<Animal> animals);
    
    float getDirectionFrom (Vec2f from, bool isPointing);
    
    static float distance (Vec2f a, Vec2f b) {
        return sqrt(
                    (a[0] - b[0]) * (a[0] - b[0]) +
                    (a[1] - b[1]) * (a[1] - b[1])
                    );
    }
    
    static Vec2f limit (Vec2f v) {
        if (norm(v) > ANIMAL_MAXFORCE) {
            return normalize(v) * ANIMAL_MAXFORCE;
        }
        return v;
    }
    
    static Vec2f pointToVec (Point2f p) {
        return Vec2f(p.x, p.y);
    }
};

#endif /* Animal_hpp */
