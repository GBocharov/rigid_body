#include "GL/glut.h"
#include "RigidBody.h"
#include "Visual.h"
#include <iostream>
#include <vector>


float last_time = 0.f;
RigidBody rigidBody = RigidBody();
RigidBody rigidBody_2 = RigidBody();

RigidBody vec_to_RBody(std::vector<double> V)
{
    RigidBody b;
    b.r = Vector{ V.at(0), V.at(1),V.at(2) };
    b.l = Vector{ V.at(3), V.at(4),V.at(5) };
    b.L = Vector{ V.at(6), V.at(7),V.at(8) };
    b.q = Quaternion{ V.at(9), V.at(10), V.at(11),  V.at(12)};

    b.I_Inv = Matrix  { V.at(13), V.at(14),V.at(15), 
                        V.at(16), V.at(17),V.at(18), 
                        V.at(19), V.at(20),V.at(21),};

    b.M_Inv = V.at(22);

    return b;
}

std::vector<double> RBody_tovector (RigidBody b)
{
    std::vector<double> res;

    res.at(0) = b.r.x;
    res.at(1) = b.r.y;
    res.at(2) = b.r.z;


    res.at(3) = b.l.x;
    res.at(4) = b.l.y;
    res.at(5) = b.l.z;

    res.at(6) = b.L.x;
    res.at(7) = b.L.y;
    res.at(8) = b.L.z;

    res.at(9) = b.q.r;
    res.at(10) = b.q.i;
    res.at(11) = b.q.j;
    res.at(12) = b.q.k;


    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res.at(i + j) = b.I_Inv.values[i][j];

    res.at(22) = b.M_Inv;

    return res;
}


RigidBody f(RigidBody& body) { //D - func

    RigidBody result;

    result.r = body.l / body.M_Inv; // v

    result.M_Inv = body.M_Inv;

    body.q = body.q.normalize();

    body.R = body.q.toMatrix();

    Vector omega = ((body.R * body.I_Inv) * body.R.transpose()) * body.L;  // w

    result.q = Quaternion{ 0.0, omega.x, omega.y, omega.z } * body.q * 0.5;

    result.l = Vector{ 0, -50 , 0 }; // t_F

    result.L = Vector{ 0, 0, 0 };      // t_torq

    result.I_Inv = body.I_Inv;

    return result;
}

template <typename T>
void solveRungeT(T& system, T(*f)(T& x), double h)
{
    T b1, b2, b3, b4;

    b1 = f(system);

    b2 = system + b1 * (h / 3);
    b2 = f(b2);

    b3 = system + ((b1 * (-h / 3)) + (b2 * h));
    b3 = f(b3);

    b4 = system + ((b1 * h) + (b2 * (-h)) + (b3 * h));
    b4 = f(b4);

    system = system + ((b1 * (1.0 / 8)) + (b2 * (3.0 / 8)) + (b3 * (3.0 / 8)) + (b4 * (1.0 / 8))) * h;
}

void solveBody(RigidBody& b, double h)
{
    solveRungeT(b, f, h);
    b.q = b.q.normalize();
    b.R = b.q.toMatrix();
}

void solveRunge(RigidBody& body, RigidBody(*f)(RigidBody& body), double h) {     
    
    RigidBody b1(rigidBody.I_Inv), b2(rigidBody.I_Inv), b3(rigidBody.I_Inv), b4(rigidBody.I_Inv); // I_Inv = const 

    b1 = f(body);

    b2 = body + b1 * (h / 3);
    b2 = f(b2);

    b3 = body + ((b1 * (-h / 3)) + (b2 * h));
    b3 = f(b3);

    b4 = body + ((b1 * h) + (b2 * (-h)) + (b3 * h));
    b4 = f(b4);

    body = body + ((b1 * (1.0 / 8)) + (b2 * (3.0 / 8)) + (b3 * (3.0 / 8)) + (b4 * (1.0 / 8))) * h;

    body.q = body.q.normalize();

    body.R = body.q.toMatrix();

    //body.print();
   // body.L.print();
    //std::cout << body.M_Inv;
}

void solve(RigidBody& body, double step, double e)
{
    double currentStep = step;
    RigidBody mainCopy = body;
    RigidBody tempCopy = body;
    Vector collisionPlacement;
    collisionPlacement = {0, 0, 0};

    bool isFindCol = false;

    solveBody(tempCopy, step);

    if (tempCopy.isCollided())
    {
       // body.L.print();
        while (1)
        {
            tempCopy = mainCopy;

            solveBody(tempCopy, currentStep);

            isFindCol = tempCopy.findCollisionPlace(e, collisionPlacement);

            if (isFindCol)
            {
                break;
            }

            if (!tempCopy.isCollided())
            {
                solveBody(mainCopy, currentStep);
            }
            else
                currentStep = currentStep / 2;
        }/**/

        body.floorCollisionAction(collisionPlacement);

       // body.L.print();
       //(body.R * collisionPlacement + body.r).print();
    }
    
    solveBody(body, currentStep);
}

void Idle() {
    glutPostRedisplay();
}


void drawFloor()
{
    glPushMatrix();
    glTranslated(0,-20,0);

    glBegin(GL_QUADS);
    glColor3d(0.5, 0.5, 0.5);
    glVertex3f(-200, -55, -850);
    glVertex3f(200, -55, -850);
    glVertex3f(200, -55, -55);
    glVertex3f(-200, -55, -55);

    glEnd();

    glPopMatrix();
}


void drawS(Vector x, double r)
{
    glColor3f(1, 0, 0);

    glPushMatrix();

    glTranslated(x.x, x.y, x.z);

    glutWireSphere(r, 10, 10);

    glPopMatrix();
}

void drawLINE(Vector x, Vector y)
{
    glColor3f(0, 1, 0);

   // glPushMatrix();

    glLineWidth(3); 
    glBegin(GL_LINES); 
    glVertex3d(x.x, x.y, x.z); 
    glVertex3d(y.x, y.y, y.z);
    glEnd();

   // glPopMatrix();
}


void drawBody() {

    for (int i = 0; i < rigidBody.Spheres.size(); i++) {
        drawS(rigidBody.Spheres.at(i).x, rigidBody.Spheres.at(i).R);

        if (i < rigidBody.Spheres.size() - 1)
            drawLINE(rigidBody.Spheres.at(i).x, rigidBody.Spheres.at(i + 1).x);
        else
            drawLINE(rigidBody.Spheres.at(i).x, rigidBody.Spheres.at(1).x);
    }

    //glColor3f(0, 1, 0);
    //glutWireSphere(50, 20, 20);
}

void drawBody1() {

    for (int i = 0; i < rigidBody.Spheres.size(); i++) {
        drawS(rigidBody.Spheres.at(i).x, rigidBody.Spheres.at(i).R);

        if (i < rigidBody.Spheres.size() - 1 && i != 3)
            drawLINE(rigidBody.Spheres.at(i).x, rigidBody.Spheres.at(i + 1).x);
    }

    //glColor3f(0, 1, 0);
    //glutWireSphere(50, 20, 20);
}


void initBody() 
{
    rigidBody.initSpheres1();
    rigidBody.findCenter();
    rigidBody.findTensor();
    rigidBody.print();
}

void Display() {
    glViewport(0, 0, 600, 600);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 1, 1, 500);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float now = clock() / (float)CLOCKS_PER_SEC;
    float dt = now - last_time;
    last_time = now;

    solve(rigidBody, dt*6, 0.000005);   //  solve (body, dt,  epsilon) 



    glPushMatrix();

    drawFloor();

    glTranslated(rigidBody.r.x, rigidBody.r.y, rigidBody.r.z - 300);

    glRotated(acos(rigidBody.q.r) * 360 / 3.14, rigidBody.q.i, rigidBody.q.j, rigidBody.q.k);

    drawBody1();

    glPopMatrix();






    glFlush();
    glutSwapBuffers();
}


int main(int argc, char* argv[]) {
    glutInit(&argc, argv);

    initBody();

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(600, 600);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("W_M");
    glutDisplayFunc(Display);
    glutIdleFunc(Idle);
    glEnable(GL_DEPTH_TEST);
    glutMainLoop();
    return 0;
}