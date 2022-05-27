#pragma once

#include "Matrix.h"
#include "Quaternion.h"
#include "Vector.h"
#include "Sphere.h"
#include <vector>


#define FLOORLEVEL -55

const double COLLIDETOLERANCE = -1.0;


 class RigidBody {
    public:
    std::vector<Sphere> Spheres;

    double M_Inv; // mass_invar

    Matrix R{}; // rotate matrix
    Vector r{}, l{}, L{}; // coord, l_momen, a_moment
    Quaternion q{}; // quat
    Matrix I_Inv;   // inetr_tens_inversed

    RigidBody();
    RigidBody(Matrix m);
    RigidBody f();
    RigidBody operator+(RigidBody A) const;
    RigidBody operator*(double h) const;

    void initSpheres();   
    void initSpheres1();  
    void findTensor();   
    void findCenter();  // find mass center
    void print();

    bool isCollided();

    bool findCollisionPlace(double e, Vector& res);
    void floorCollisionAction(Vector collisionPlace);


};
