#include "RigidBody.h"
#include <iostream>


RigidBody RigidBody::operator+(RigidBody A) const {

    RigidBody result;
    result.r = r + A.r;
    result.q = q + A.q;
    result.l = l + A.l;
    result.L = L + A.L;

    result.Spheres = Spheres;

    result.I_Inv = I_Inv;

    result.M_Inv = M_Inv;

    return result;
}

RigidBody RigidBody::operator*(double h) const {

    RigidBody result{};

    result.r = r * h;

    result.q = q * h;

    result.l = l * h;

    result.L = L * h;

    result.Spheres = Spheres;

    result.I_Inv = I_Inv;

    result.M_Inv = M_Inv;

    return result;
}

RigidBody::RigidBody() {
  
    r = { 0,0,0 };
    q = { cos(0), 0, 0, 0 };

    R = q.normalize().toMatrix();
   
    l = { 0 , 0,0 };
    L = Vector{ 0 ,10000, 0 };

    //I_Inv.printMatr();

}

RigidBody::RigidBody(Matrix m) {
    I_Inv = m;
    //I_Inv.printMatr();
}



void RigidBody::initSpheres()
{
    Vector x1 = Vector{ 0, 0, 0 }*50;
    double R1 = 7;
    double mass1 = 10;
    Sphere s1 = Sphere(x1, mass1, R1);


    Vector x2 = Vector{ 1, 0, 0 }*50;
    double R2 = 7;
    double mass2 = 10;
    Sphere s2 = Sphere(x2, mass2, R2);

    Vector x3 = Vector{ 1, 2, 0 }*50;
    double R3 = 7;
    double mass3 = 10;
    Sphere s3 = Sphere(x3, mass3, R3);

    Vector x4 = Vector{ 2, 2, 0 }*50;
    double R4 = 7;
    double mass4 = 10;
    Sphere s4 = Sphere(x4, mass4, R4);


    Vector x5 = Vector{ 2, 0, 0 }*50;
    double R5 = 7;
    double mass5 = 10;
    Sphere s5 = Sphere(x5, mass5, R5);


    Vector x6 = Vector{ 2, 1, 0 }*50;
    double R6 = 7;
    double mass6 = 10;
    Sphere s6 = Sphere(x6, mass6, R6);

    Vector x7 = Vector{ 0, 1, 0 }*50;
    double R7 = 7;
    double mass7 = 10;
    Sphere s7 = Sphere(x7, mass7, R7);

    Vector x8 = Vector{ 0, 2, 0 }*50;
    double R8 = 7;
    double mass8 = 10;
    Sphere s8 = Sphere(x8, mass8, R8);

    Spheres.push_back(s1);
    Spheres.push_back(s2);
    Spheres.push_back(s3);
    Spheres.push_back(s4);
    Spheres.push_back(s5);
    Spheres.push_back(s6);
    Spheres.push_back(s7);
    Spheres.push_back(s8);

}


void RigidBody::initSpheres1()
{

    Vector x1 = Vector{ 2, 2, 2 }*10;
    double R1 = 7;
    double mass1 = 10;
    Sphere s1 = Sphere(x1, mass1, R1);


    Vector x2 = Vector{ 0, 0, 0 }*10;
    double R2 = 7;
    double mass2 = 10;
    Sphere s2 = Sphere(x2, mass2, R2);

    Vector x3 = Vector{ 0, 0, 4 }*10;
    double R3 = 7;
    double mass3 = 10;
    Sphere s3 = Sphere(x3, mass3, R3);

    Vector x4 = Vector{ 4, 0, 0 }*10;
    double R4 = 7;
    double mass4 = 10;
    Sphere s4 = Sphere(x4, mass4, R4);


    Vector x5 = Vector{ 4, 0, 4 }*10;
    double R5 =7;
    double mass5 = 10;
    Sphere s5 = Sphere(x5, mass5, R5);


    Vector x6 = Vector{ 0, 4, 0 }*10;
    double R6 = 7;
    double mass6 = 10;
    Sphere s6 = Sphere(x6, mass6, R6);

    Vector x7 = Vector{ 0, 4, 4 }*10;
    double R7 = 7;
    double mass7 = 10;
    Sphere s7 = Sphere(x7, mass7, R7);

    Vector x8 = Vector{ 4, 4, 0 }*10;
    double R8 = 7;
    double mass8 = 10;
    Sphere s8 = Sphere(x8, mass8, R8);


    Vector x9 = Vector{ 4, 4, 4 }*10;
    double R9 = 7;
    double mass9 = 10;
    Sphere s9 = Sphere(x9, mass9, R9);

    // test1 cube
   {

    Spheres.push_back(s1);
    Spheres.push_back(s2);
    Spheres.push_back(s3);
    Spheres.push_back(s4);
    Spheres.push_back(s5);
    Spheres.push_back(s6);
    Spheres.push_back(s7);
    Spheres.push_back(s8);
    Spheres.push_back(s9);

    }/**/


    //test2
   /* {
        Spheres.push_back(s2);
        Spheres.push_back(s4);
    }

    {
        Spheres.push_back(s2);
        Spheres.push_back(s3);
        Spheres.push_back(s5);
        Spheres.push_back(s6);
        Spheres.push_back(s7);
        Spheres.push_back(s9);
    }*/
}

void RigidBody::findCenter() 
{
    double total_mass = 0;
    Vector x = {0, 0, 0};

    for (auto &i : Spheres) 
    {
        x = x + i.x * i.mass;
        total_mass += i.mass;
       // x.print();
    }

    x = x / total_mass;

    M_Inv = total_mass;

    std::cout << "Center mass = ";
    //std::cout << M_Inv;
    x.print();

    r = x;

    for (auto& i : Spheres) //Trans Spheres 
    {
        i.x = i.x - x;
    }

}


void RigidBody::findTensor() 
{

    for (auto& i : Spheres) 
    {
       // std::cout << "DO:" << std::endl;
       // i.I.printMatr();

        i.Compute({ 0.0, 0.0, 0.0 });

      //  std::cout << "Posle:" << std::endl;
      //  i.I.printMatr();
    }

    for (auto& i : Spheres) 
    {
        I_Inv = I_Inv + i.I;
    }

    std::cout << "Final inertia tensor:" << std::endl;

    I_Inv.printMatr();

    I_Inv = I_Inv.reverse();
    
    std::cout << "Final inertia tensor inverse:" << std::endl;

    I_Inv.printMatr();
}

void RigidBody::print()
{
    std::cout << "-------------------" << std::endl;
    std::cout << "Mass = "<< M_Inv<<std::endl;
    std::cout << "x = ";
    r.print();
    std::cout << "l = ";

    l.print();
    std::cout << "L = ";

    L.print();


    std::cout << "I_Inv = "<<std::endl;

    I_Inv.printMatr();


    std::cout << "-------------------" << std::endl;
}



bool RigidBody::isCollided()

{

    double collideTolerance = COLLIDETOLERANCE;
    double floorLevel = FLOORLEVEL;
    Vector n = { 0, 1, 0 };

    R = q.normalize().toMatrix();
    for (auto &i : Spheres)
    {
        Vector collisionPlace = R * i.x;
        Matrix inertiaTensorInv = R * I_Inv * R.transpose();
        Vector p_dot = l / M_Inv + ((I_Inv * L)*(collisionPlace));

        bool isBelowFloor = ((collisionPlace + r ).dot(n) - floorLevel < i.R);

        bool isGoesToFloor = p_dot.dot(n) < -collideTolerance;

        if (isBelowFloor)
        {
            return true;
        }
    }

    return false;

}


bool RigidBody::findCollisionPlace(double e, Vector& res)
{
    double floorLevel = FLOORLEVEL;
    double final_dist = 10000000000;
    int k = 0;
    double final_R = 0;
    Vector final_coord;
    for (auto& i : Spheres) // chose most collised sphere
    {
        double distance = (R * i.x + r).y - floorLevel - i.R;



      /*  if (abs(distance) < e )
        {
            k = 1;
            if (distance < final_dist) 
            {
                final_dist = distance;
                final_coord = i.x;
                final_R = i.R;
            }
        }*/

        
        if (distance < final_dist)
        {
            final_dist = distance;
            final_coord = i.x;
            final_R = i.R;

            if (abs(distance) < e)
            {
                k = 1;
            }
        }

    }

    if (k == 1) {
        std::cout << "Collision coord_W --> " << std::endl; // collision coord 
        ((R * final_coord + r+ Vector{ 0, -final_R, 0 })).print(); // vector in W.coord
        res =  (final_coord + R.transpose() * Vector { 0, -final_R, 0 });
        return true;
    }
    return false;
}

void RigidBody::floorCollisionAction(Vector collisionPlace)
{
    
    Vector coll_Pl_Rotate = R * collisionPlace;
    Matrix inertiaTensorInv = R * I_Inv * R.transpose();
    Vector p_dot = l / M_Inv + ((inertiaTensorInv * L)* coll_Pl_Rotate);
    Vector n;
    n = { 0, 1, 0 };
    double numerator = (-1 * n.dot(p_dot));
    double term1 = 1 / M_Inv;
    double term2 = n.dot((inertiaTensorInv * (coll_Pl_Rotate *(n)))*(coll_Pl_Rotate));
    Vector total_force =   n*( 2 * (numerator / (term1 + term2)));
    l = l + total_force;
    L = L + coll_Pl_Rotate *(total_force);
}


