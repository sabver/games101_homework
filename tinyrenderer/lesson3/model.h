#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"

class Model {
private:
	std::vector<Vec3f> verts_;
	std::vector<std::vector<int> > faces_;
	
	std::vector<Vec3f> vert_textures_;
	std::vector<std::vector<int> > face_textures_;
public:
	Model(const char *filename);
	~Model();
	int nverts();
	int nfaces();
	int nvert_textures();
	int nface_textures();
	Vec3f vert(int i);
	std::vector<int> face(int idx);
	Vec3f vert_texture(int i);
	std::vector<int> face_texture(int idx);
};

#endif //__MODEL_H__