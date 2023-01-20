#ifndef HITTABLE_LIST_H
#define HITTABLE_LIST_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include "rtweekend.h"

#include "hittable.h"

#include <memory>
#include <vector>
#include <list>

class hittable_list : public hittable  {
    public:
        hittable_list(bool _light=false) {light = _light;}
        hittable_list(shared_ptr<hittable> object,bool _light=false) { 
            add(object);
            light = _light;
            if(light)
                generate_weights();
        }

        void clear() { 
            objects.clear();
            if(light)
                generate_weights();
        }
        void add(shared_ptr<hittable> object) { 
            objects.push_back(object); 
            if(light)
                generate_weights();
        }

        virtual bool hit(
            const ray& r, double t_min, double t_max, hit_record& rec) const override;

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;
        virtual double pdf_value(const vec3 &o, const vec3 &v) const override;
        virtual vec3 random(const vec3 &o) const override;

    private:
        void generate_weights();
        int random_select() const;

    public:
        std::vector<shared_ptr<hittable>> objects;
        std::list<double> weights;
        std::list<double> accu;
        bool light;
};


bool hittable_list::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    hit_record temp_rec;
    auto hit_anything = false;
    auto closest_so_far = t_max;

    for (const auto& object : objects) {
        if (object->hit(r, t_min, closest_so_far, temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }

    return hit_anything;
}


bool hittable_list::bounding_box(double time0, double time1, aabb& output_box) const {
    if (objects.empty()) return false;

    aabb temp_box;
    bool first_box = true;

    for (const auto& object : objects) {
        if (!object->bounding_box(time0, time1, temp_box)) return false;
        output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
        first_box = false;
    }

    return true;
}


double hittable_list::pdf_value(const point3& o, const vec3& v) const {
    auto sum = 0.0;
    auto weight_it = weights.begin();

    for (int i=0;i<objects.size();i++){
        sum += (*weight_it) * objects[i]->pdf_value(o, v);
        weight_it++;
    }
    return sum;
}


vec3 hittable_list::random(const vec3 &o) const {
    auto int_size = static_cast<int>(objects.size());
    return objects[random_select()]->random(o);
}

void hittable_list::generate_weights() {
    std::cerr << "generate_weights for lights\n";
    weights = std::list<double>();
    accu = std::list<double>();
    if(objects.size() == 0) return;
    double tot=0;
    for(auto obj : objects){
        weights.push_back(1 / obj->area());
        tot += 1 / obj->area();
        accu.push_back(tot);
    }
    auto accu_it = accu.begin();
    auto weights_it = weights.begin();
    for(int i=0; i<objects.size();i++){
        *accu_it /= tot;
        *weights_it /= tot;
        accu_it++;
        weights_it++;
    }
}

int hittable_list::random_select() const {
    double rand_real = random_double();
    auto accu_it = accu.begin();
    for(int i=0;i<objects.size();i++){
        if(*accu_it > rand_real){
            return i;
        }
        accu_it++;
    }
    return objects.size() - 1;
}

#endif
