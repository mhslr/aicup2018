#pragma once
#include "parameters.cpp"
#include <bits/stdc++.h>
using namespace std;

struct p2 {
    double x,z;

    p2 () {} 
    p2 (const double& _x, const double& _z)
        : x(_x), z(_z) {}

    double norm()
    { return sqrt( x*x + z*z ); }
    double norm2()
    { return x*x + z*z ; }

    p2 operator+(const p2& o) const
    { return {x+o.x, z+o.z}; }
    p2 operator-(const p2& o) const
    { return {x-o.x, z-o.z}; }
    p2 operator*(const double& k) const
    { return {k*x, k*z}; }
    p2& operator+=(const p2& o)
    { *this = (*this)+o; return *this; }
    p2& operator-=(const p2& o)
    { *this = (*this)-o; return *this; }
    p2& operator*=(const double& k)
    { *this = (*this)*k; return *this; }
};

double cross (const p2& a, const p2& b)
{ return a.x*b.z - a.z*b.x; }
double  dot  (const p2& a, const p2& b)
{ return a.x*b.x + a.z*b.z; }

// probably useless if we only care about (z=...) planes 
struct segment {
    p2 o,d; // origin and direction
    segment (const double& xo, const double& zo,
             const double& xd, const double& zd)
        : o(xo,zo), d(xd,zd) {}
};
p2 intersect(const segment& s1,const segment& s2) {
    double kek = cross(s1.d,s2.d);
    if (!kek) return {0.0,0.0}; // we want to avoid crash
    return s1.o + s1.d * (cross(s2.o-s1.o,s2.d) / cross(s1.d,s2.d));
}

struct p3 {
    double x,y,z;

    p3 () {} 
    p3 (const double& x, const double& y, const double& z)
        : x(x), y(y), z(z) {}

    double norm()
    { return sqrt( x*x + y*y + z*z ); }
    double norm2()
    { return x*x + y*y + z*z ; }
    p2 flat() { return {x,z}; }
    double flatnorm()
    { return sqrt( x*x + z*z ); }
    double flatnorm2()
    { return x*x + z*z ; }

    p3 operator+(const p3& o) const
    { return {x+o.x, y+o.y, z+o.z}; }
    p3 operator-(const p3& o) const
    { return {x-o.x, y-o.y, z-o.z}; }
    p3 operator*(const double& k) const
    { return {k*x, k*y, k*z}; }
    p3& operator+=(const p3& o)
    { *this = (*this)+o; return *this; }
    p3& operator-=(const p3& o)
    { *this = (*this)-o; return *this; }
    p3& operator*=(const double& k)
    { *this = (*this)*k; return *this; }
};

double  dot  (const p3& a, const p3& b)
{ return a.x*b.x + a.y*b.y + a.z*b.z; }

struct traj { // parabolic trajectory
    double x,y,z; // tip of parabola
    double vx,vz; // plane velocity
    double ttt; // time till tiptop

    traj () {}
    // constructor from cur pos and velocity
    traj(const p3& pos,const p3& vel) {
        ttt = vel.y/GRAV;
        x = pos.x + ttt*vel.x;
        y = pos.y + ttt*vel.y - ttt*ttt*GRAV/2; // <3 physics
        z = pos.z + ttt*vel.z;
        vx = vel.x;
        vz = vel.z;
    }

    // predict pos and vel in t seconds
    pair<p3,p3> predict(double t) {
        t -= ttt; // how much time from tiptop
        p3 pos (x+t*vx, y-t*t*GRAV/2,z+t*vz);
        p3 vel (vx, -t*GRAV, vz);
        // cannot go below earth
        if (pos.y < BRAD) {
            pos.y = BRAD;
            vel.y = 0.0;
        }
        if (fabs(pos.x) > AWID-BRAD) {
            pos.x = (pos.x > 0.0 ? 2 : -2) * (AWID-BRAD) - pos.x;
            vel.x = -vel.x;
        }
        if (pos.z > ADEP-BRAD) {
            pos.z = 2*(ADEP-BRAD) - pos.z;
            vel.z = -vel.z;
        }
        return {pos,vel};
    }
};

// velocities of robot and ball after collision
pair<p3,p3> collide (
        const p3& Prob, const p3& Vrob,
        const p3& Pbal, const p3& Vbal) {
    // normal vector to collision surface
    p3 delta = Prob-Pbal;
    double dn = delta.norm();
    // assert dn approx BRAD + RRAD
    delta *= (1/dn);
    // initial
    double uR = dot(Vrob,delta),
           uB = dot(Vbal,delta);
    // outogoing
    double vR = ((MROB-MBAL)*uR + 2*MBAL*uB) / (MROB+MBAL),
           vB = ((MBAL-MROB)*uB + 2*MROB*uR) / (MROB+MBAL);
    p3 Wrob = Vrob + delta*(vR-uR);
    p3 Wbal = Vbal + delta*(vB-uB);
    return {Wrob,Wbal};
}
