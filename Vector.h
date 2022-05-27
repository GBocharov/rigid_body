#ifndef WITCHMATH_VECTOR_H
#define WITCHMATH_VECTOR_H


struct Vector {
    double x;
    double y;
    double z;

    Vector operator*(const Vector& other) const;
    double dot(const Vector& other) const;
    Vector operator+(const Vector& other) const;
    Vector operator-(const Vector& other) const;
    Vector operator*(const double d) const;
    Vector operator/(const double d) const;

    void print();
};


#endif //WITCHMATH_VECTOR_H