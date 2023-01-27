#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

// First attempt
// void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
//     for (float t=0.; t<1.; t+=.1) {
//         int x = x0*(1.-t) + x1*t;
//         int y = y0*(1.-t) + y1*t;
//         image.set(x, y, color);
//     }
// }

// Second attempt
// void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) { 
//     for (int x=x0; x<=x1; x++) { 
//         // x是每次加1对应一个像素，然后t是计算出当前x走了百分比
//         float t = (x-x0)/(float)(x1-x0); 
//         int y = y0*(1.-t) + y1*t; 
//         image.set(x, y, color); 
//     } 
// }

// Third attempt
// void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) { 
//     bool steep = false; 
//     // 决定哪个轴来进行++1，通常要由长的轴来做++1，要不然会看到很多洞
//     if (std::abs(x0-x1)<std::abs(y0-y1)) { // if the line is steep, we transpose the image 
//         std::swap(x0, y0); 
//         std::swap(x1, y1); 
//         steep = true; 
//     } 
//     if (x0>x1) { // make it left−to−right 
//         std::swap(x0, x1); 
//         std::swap(y0, y1); 
//     } 
//     for (int x=x0; x<=x1; x++) { 
//         float t = (x-x0)/(float)(x1-x0); 
//         int y = y0*(1.-t) + y1*t; 
//         if (steep) { 
//             image.set(y, x, color); // if transposed, de−transpose 
//         } else { 
//             image.set(x, y, color); 
//         } 
//     } 
// }

// Fourth attempt continued
// void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) { 
//     bool steep = false; 
//     if (std::abs(x0-x1)<std::abs(y0-y1)) { 
//         std::swap(x0, y0); 
//         std::swap(x1, y1); 
//         steep = true; 
//     } 
//     if (x0>x1) { 
//         std::swap(x0, x1); 
//         std::swap(y0, y1); 
//     } 
//     int dx = x1-x0; 
//     int dy = y1-y0;
//     // y = ax + b, 然后每次y1 - y2 = a(x1 - x2),这里x1 - x2 = 1，所以是y1 - y2 = a
//     // 也就是说，x每增加1，y相对增加a
//     float derror = std::abs(dy/float(dx)); 
//     float error = 0; 
//     int y = y0; 
//     for (int x=x0; x<=x1; x++) { 
//         if (steep) { 
//             image.set(y, x, color); 
//         } else { 
//             image.set(x, y, color); 
//         } 
//         // 这里error记录过往已经积累了的增长值，如果增长值积累超过0.5了，那就y增长，否则就等待
//         error += derror; 
//         // 但是，因为y每次只能+1(-1)的幅度进行增长，所以有时候不一定会进行增长
//         // 它可能维持好几次的不增长然后后面才+1来表达不同的增长速率
//         if (error>.5) { 
//             y += (y1>y0?1:-1); 
//             error -= 1.; 
//         } 
//     } 
// } 

// Timings: fifth and final attempt
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) { 
    bool steep = false; 
    if (std::abs(x0-x1)<std::abs(y0-y1)) { 
        std::swap(x0, y0); 
        std::swap(x1, y1); 
        steep = true; 
    } 
    if (x0>x1) { 
        std::swap(x0, x1); 
        std::swap(y0, y1); 
    } 
    int dx = x1-x0; 
    int dy = y1-y0; 
    // 这里为了解决出现除法的问题，把下面的几行代码改为了只有加减的版本，下面的代码运行的结果一致，只是做了一个替换
    // 替换的思路只要在error2 > dx这里。把0.5换成dx就可以理解了，数学上等价
    // float derror = std::abs(dy/float(dx)); 
    int derror2 = std::abs(dy)*2; 
    // float error = 0; 
    int error2 = 0; 
    int y = y0; 
    for (int x=x0; x<=x1; x++) { 
        if (steep) { 
            image.set(y, x, color); 
        } else { 
            image.set(x, y, color); 
        } 
        error2 += derror2; 
        // if (error>.5) { 
        if (error2 > dx) { 
            y += (y1>y0?1:-1); 
            // error -= 1.; 
            error2 -= dx*2; 
        } 
    } 
} 

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        for (int j=0; j<3; j++) {
            Vec3f v0 = model->vert(face[j]);
            Vec3f v1 = model->vert(face[(j+1)%3]);
            int x0 = (v0.x+1.)*width/2.;
            int y0 = (v0.y+1.)*height/2.;
            int x1 = (v1.x+1.)*width/2.;
            int y1 = (v1.y+1.)*height/2.;
            line(x0, y0, x1, y1, image, white);
        }
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

