//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

/**
 * @brief 
 * 在场景的所有光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度。
 * 这里好像默认场景中只有一个光源，然后取在这个光源的采样点和pdf
 * @param pos 
 * @param pdf 
 */
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
// 伪代码里面的wo和wi的方向定义是什么？
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // 点p 
    Intersection obj_pos = intersect(ray);   

    // 如果没有hit到物体，那就直接结束
    if( obj_pos.happened == false ){
        return Vector3f(0.0, 0.0, 0.0);
    }
    // 如果是打中光源的话，那就直接返回
    if( obj_pos.m->hasEmission() ){
        return obj_pos.m->getEmission();
    }

    float light_pdf;
    Intersection light_pos;
    // 从光源点进行采样，采样出来的点有可能没法和点p相同（中间有阻碍物）
    sampleLight(light_pos, light_pdf);

    // obj_pos 当前光线追踪的点p
    // light_pos 光源的接触点
    // wo 从点p出射到相机的向量
    // wi 从另外一个物体q出射到点p的向量
    // ws 从光源点出射到点p的向量
    // N 点p的Normal
    // NN 光源点的Normal

    // 光源的点是light_pos.coords, 追踪的物体的坐标是obj_pos.coords, ws是从光源点出发到物体坐标的向量
    Vector3f ws = (obj_pos.coords - light_pos.coords).normalized();
    Vector3f wo = -ray.direction;
    Vector3f N = obj_pos.normal.normalized(), NN = light_pos.normal.normalized();
    float ws_distance = ws.norm();

    // TODO 这里需要判断一个逻辑，这两点之间是否有物体遮挡
    float EPLISON = 0.0001;
    Ray ray_test(obj_pos.coords, -ws);
    Intersection ws_ray_inter = intersect(ray_test);

    // 直接光照
    // emit * eval() * dot(ws, N) * dot(ws, NN)/|x-p|^2/pdf_light
    Vector3f L_dir = Vector3f(0.0, 0.0, 0.0);
    if(ws_ray_inter.distance - ws_distance > -EPLISON) {
       L_dir = light_pos.emit * obj_pos.m->eval(ws, wo, N) * dotProduct(ws, NN) * dotProduct(-ws, N) / std::pow(ws_distance, 2) / light_pdf; 
    }
    
    // 俄罗斯转盘做法
    float ksi = get_random_float();
    if( ksi > RussianRoulette ){
        return Vector3f(0.0, 0.0, 0.0);
    }

    Vector3f L_indir = Vector3f(0.0, 0.0, 0.0);

    // 光线打到物体之后的出射向量
    Intersection obj_output_pos;
    // 采样拿到wi和pdf，这里想法是反过来，从点p采样出来的光线是从点q出发射到点p的，所以是wi
    // 这里的点q还没有通过intersect方法查找
    Vector3f wi = -obj_pos.m->sample(ray.direction, N);

    // 需要判断点p出射出去光线是否碰到一个non-emitting的物体，如果碰到光源呢？
    // 重新构建一条光线，从点p朝着wi的方向走，这里要注意一下wi的方向
    Ray p_ray(obj_pos.coords, -wi);
    Intersection q_pos = Scene::intersect(p_ray);

    if( q_pos.happened ){
        float obj_pdf = obj_pos.m->pdf(wi, wo, N);
        // 这里判断不是光源，问题是，如果是遇到光源呢？
        if( q_pos.m->hasEmission() == false ){
            // shade(q, wi)*eval(wo, wi, N)*dot(wi, N)/pdf(wo, wi, N)/RussianRoulette
            L_indir = this->castRay(p_ray, depth+1) * obj_pos.m->eval(wi, wo, N) * dotProduct(-wi, obj_pos.normal) / obj_pdf / RussianRoulette;         
        }
    }
    return L_dir + L_indir;
}