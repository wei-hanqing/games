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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = intersect(ray);
    Material *m = intersection.m;
    Vector3f L_dir = Vector3f(0.0,0.0,0.0);
    Vector3f L_indir = Vector3f(0.0,0.0,0.0);

    if(intersection.happened) {
        if(intersection.m->hasEmission())
        {
            if(depth==0) return intersection.m->getEmission();
            else return L_dir;
        }
        Vector3f hitPoint = intersection.coords;
        Vector3f N = normalize(intersection.normal); // normal

        // contribution from the light source
        Intersection inter;
        float pdf_light = 0.0;
        sampleLight(inter, pdf_light);
        Ray px(hitPoint, normalize(inter.coords - hitPoint)); 
        Intersection tmp_point = intersect(px);
        float dis = (hitPoint-inter.coords).norm();
        if(tmp_point.happened && (abs(dis-tmp_point.distance) < 100.0*EPSILON))
        {
            L_dir = inter.emit * m->eval(ray.direction, px.direction, N) * dotProduct(px.direction, N) 
                    * dotProduct(-px.direction, normalize(inter.normal)) / (dis*dis) / pdf_light;
        }

        // contribution from other reflectors
        float ksi = get_random_float();
        if(ksi < RussianRoulette)
        {
            Vector3f wi = normalize(m->sample(ray.direction, N));
            Ray r(hitPoint, wi);
            Intersection tmp_point = intersect(r);
            if(tmp_point.happened && !tmp_point.m->hasEmission())   // hit object and object is not light
            {
                L_indir = castRay(r, depth+1) * m->eval(ray.direction, wi, N) * dotProduct(wi, N) / m->pdf(ray.direction, wi, N) / RussianRoulette;
            }
        }

    }
    return L_dir + L_indir;

}