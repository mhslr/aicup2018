#pragma once
#include "parameters.cpp" // for constants in trajectories
#include <bits/stdc++.h>
using namespace std;

struct p2 {
    double x,z;

    double norm()
    { return sqrt( x*x + z*z ); }
    double norm2()
    { return x*x + z*z ; }

    p2 operator+(const p2& o)
    { return {x+o.x, z+o.z}; }
    p2 operator-(const p2& o)
    { return {x-o.x, z-o.z}; }
    p2 operator*(const double& k)
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

struct seg {
    p2 o,d; // origin and direction
    seg (const double& xo, const double& zo,
         const double& xd, const double& zd)
        : o(xo,zo),d(xd,zd) {}
};

p2 intersect(const seg& s1,const seg& s2) {
    double kek = cross(s1.d,s2.d);
    if (!kek) return {0.0,0.0}; // we want to avoid crash
    return s1.o + s1.d * (cross(s2.o-s1.o,s2.d) / cross(s1.d,s2.d));
}

struct p3 {
    double x,y,z;

    double norm()
    { return sqrt( x*x + y*y + z*z ); }
    double norm2()
    { return x*x + y*y + z*z ; }

    p2 flat() { return {x,z}; }

    p3 operator+(const p3& o)
    { return {x+o.x, y+o.y, z+o.z}; }
    p3 operator-(const p3& o)
    { return {x-o.x, y-o.y, z-o.z}; }
    p3 operator*(const double& k)
    { return {k*x, k*y, k*z}; }
    p3& operator+=(const p3& o)
    { *this = (*this)+o; return *this; }
    p3& operator-=(const p3& o)
    { *this = (*this)-o; return *this; }
    p3& operator*=(const double& k)
    { *this = (*this)*k; return *this; }
};

struct traj { // parabolic trajectory
    double x,y,z; // tip of parabola
    double vx,vz; // plane velocity
    double ttt; // time till tiptop

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
        return {pos,vel};
    }
};
