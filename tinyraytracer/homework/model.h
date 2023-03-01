#ifndef __MODEL_H__
#define __MODEL_H__
#include <vector>
#include <string>
#include "material.h"
#include "mesh.h"
#include "geometry.h"

class Model : public Mesh
{
private:
    std::vector<Vec3f> verts;
    std::vector<Vec3i> faces;

public:
    Model(const char *filename, Material m);
    Vec3f bbox_min;
    Vec3f bbox_max;

    int nverts() const; // number of vertices
    int nfaces() const; // number of triangles

    // ray_triangle_intersect
    // Moller and Trumbore
    bool ray_triangle_intersect(const int &fi, const Vec3f &orig, const Vec3f &dir, float &tnear) const
    {
        Vec3f edge1 = point(vert(fi, 1)) - point(vert(fi, 0));
        Vec3f edge2 = point(vert(fi, 2)) - point(vert(fi, 0));
        Vec3f pvec = cross(dir, edge2);
        float det = edge1 * pvec;
        if (det < 1e-5)
            return false;

        Vec3f tvec = orig - point(vert(fi, 0));
        float u = tvec * pvec;
        if (u < 0 || u > det)
            return false;

        Vec3f qvec = cross(tvec, edge1);
        float v = dir * qvec;
        if (v < 0 || u + v > det)
            return false;

        tnear = edge2 * qvec * (1. / det);
        return tnear > 1e-5;
    };

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &tnear, Vec3f &N) const override
    {
        if (is_intersect(orig, dir))
        {
            bool isIntersection = false;
            int flagFi = -1;
            float t = std::numeric_limits<float>::max();
            for (int fi = 0; fi < (int)nfaces(); fi++)
            {
                if (ray_triangle_intersect(fi, orig, dir, tnear) && t > tnear)
                {
                    t = tnear;
                    isIntersection = true;
                    flagFi = fi;
                }
            }
            if (isIntersection)
            {
                // calculate normal
                Vec3f edge1 = point(vert(flagFi, 1)) - point(vert(flagFi, 0));
                Vec3f edge2 = point(vert(flagFi, 2)) - point(vert(flagFi, 0));
                N = cross(edge1, edge2);
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    const Vec3f &point(int i) const;       // coordinates of the vertex i
    Vec3f &point(int i);                   // coordinates of the vertex i
    int vert(int fi, int li) const;        // index of the vertex for the triangle fi and local index li
    void get_bbox(Vec3f &min, Vec3f &max); // bounding box for all the vertices, including isolated ones
    bool is_intersect(const Vec3f &orig, const Vec3f &dir) const
    {
        float t_min_x = (bbox_min.x - orig.x) / dir.x;
        float t_min_y = (bbox_min.y - orig.y) / dir.y;
        float t_min_z = (bbox_min.z - orig.z) / dir.z;

        float t_max_x = (bbox_max.x - orig.x) / dir.x;
        float t_max_y = (bbox_max.y - orig.y) / dir.y;
        float t_max_z = (bbox_max.z - orig.z) / dir.z;

        if (dir.x < 0)
        {
            std::swap(t_max_x, t_min_x);
        }
        if (dir.y < 0)
        {
            std::swap(t_max_y, t_min_y);
        }
        if (dir.z < 0)
        {
            std::swap(t_max_z, t_min_z);
        }

        float t_enter = std::max(t_min_x, std::max(t_min_y, t_min_z));
        float t_exit = std::min(t_max_x, std::min(t_max_y, t_max_z));
        return t_exit > t_enter && t_exit >= 0;
    };
};

std::ostream &operator<<(std::ostream &out, Model &m);

#endif //__MODEL_H__
