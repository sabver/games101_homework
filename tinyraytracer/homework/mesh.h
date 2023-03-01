#ifndef __MESH_H__
#define __MESH_H__

#include "geometry.h"
class Mesh
{
public:
    virtual bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &tnear, Vec3f &N) const = 0;
    Material material;
};

#endif
