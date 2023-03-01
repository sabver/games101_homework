#ifndef __SPHERE_H__
#define __SPHERE_H__

#include "material.h"
#include "mesh.h"
#include "geometry.h"

class Sphere : public Mesh
{

public:
    Vec3f center;
    float radius;    
    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r) {
        material = m;
    }    
    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0, Vec3f &N) const override
    {
        Vec3f L = center - orig;
        float tca = L * dir;
        float d2 = L * L - tca * tca;
        if (d2 > radius * radius)
            return false;
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        // setting normal
        N = (orig + dir * t0 - center).normalize();
        return true;
    }
};

#endif
