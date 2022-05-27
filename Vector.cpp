#include "Vector.h"
#include <iostream>


Vector Vector::operator*(const Vector& other) const {
    double i = y * other.z - z * other.y;
    double j = z * other.x - x * other.z;
    double k = x * other.y - y * other.x;
    return Vector{ i, j, k };
}

Vector Vector::operator*(const double d) const {
    double i = x * d;
    double j = y * d;
    double k = z * d;
    return Vector{ i, j, k };
}

double Vector::dot(const Vector& other) const {
    double i = x * other.x;
    double j = y * other.y;
    double k = z * other.z;
    return  (i + j + k) ;
}
Vector Vector::operator/(const double d) const {
    double i = x / d;
    double j = y / d;
    double k = z / d;
    return Vector{ i, j, k };
}

Vector Vector::operator+(const Vector& other) const {
    double i = x + other.x;
    double j = y + other.y;
    double k = z + other.z;
    return Vector{ i, j, k };
}
Vector Vector::operator-(const Vector& other) const {
    double i = x - other.x;
    double j = y - other.y;
    double k = z - other.z;
    return Vector{ i, j, k };
}

void Vector::print() 
{
    std::cout << "{ " << x << " " << y << " " << z << "}" << std::endl;
}