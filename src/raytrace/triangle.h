#ifndef TRIANGLE_H
#define TRIANGLE_H
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
#include "vec3.h"




class triangle : public hittable {
    public:
        triangle() {}

        triangle(
            point3 _p1, point3 _p2, point3 _p3, shared_ptr<material> mat
        ) : p1(_p1), p2(_p2), p3(_p3), mp(mat) {
            double a = (p1 - p2).length();
            double b = (p1 - p3).length();
            double c = (p2 - p3).length();
            double p = a + b + c;
            triangle_area = sqrt(p * (p - a) * (p - b) * (p - c));
        };

        virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
            // The bounding box must have non-zero width in each dimension, so pad the Y
            // dimension a small amount.
            double xmin, xmax, ymin, ymax, zmin, zmax;
            xmin = fmin(fmin(p1.x(), p2.x()), p3.x()) - 0.0001;
            ymin = fmin(fmin(p1.y(), p2.y()), p3.y()) - 0.0001;
            zmin = fmin(fmin(p1.z(), p2.z()), p3.z()) - 0.0001;
            xmax = fmax(fmax(p1.x(), p2.x()), p3.x()) + 0.0001;
            ymax = fmax(fmax(p1.y(), p2.y()), p3.y()) + 0.0001;
            zmax = fmax(fmax(p1.z(), p2.z()), p3.z()) + 0.0001;
            output_box = aabb(point3(xmin, ymin, zmin), point3(xmax, ymax, zmax));
            return true;
        }

        // virtual double pdf_value(const point3& origin, const vec3& v) const override {
        //     hit_record rec;
        //     if (!this->hit(ray(origin, v), 0.001, infinity, rec))
        //         return 0;

        //     auto area = triangle_area;
        //     auto distance_squared = rec.t * rec.t * v.length_squared();
        //     auto cosine = fabs(dot(v, rec.normal) / v.length());

        //     return distance_squared / (cosine * area);
        // }

        // virtual vec3 random(const point3& origin) const override {
        //     auto random_point = point3(random_double(x0,x1), k, random_double(z0,z1));
        //     return random_point - origin;
        // }

    public:
        shared_ptr<material> mp;
        point3 p1, p2, p3;
        double triangle_area;
};



bool triangle::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    auto normal = unit_vector(cross(p1 - p2, p1 - p3));

    if(fabs(dot(normal, unit_vector(r.direction()))) < 0.0001)
        return false;
    
    // Moller Trumbore Algorithm
    auto e1 = p2 - p1;
    auto e2 = p3 - p1;
    auto s = r.origin() - p1;
    auto s1 = cross(r.direction(), e2);
    auto s2 = cross(s, e1);

    auto deno = dot(s1, e1);
    auto t = dot(s2, e2) / deno;
    auto b1 = dot(s1, s) / deno;
    auto b2 = dot(s2, r.direction()) / deno;

    if(t < t_min || t > t_max)
        return false;
    
    if(!(b1 > 0 && b2 > 0 && 1 - b1 - b2 > 0))
        return false;

    rec.u = 0.5;
    rec.v = 0.5;
    rec.t = t;
    auto outward_normal = normal;
    rec.set_face_normal(r, outward_normal);
    rec.mat_ptr = mp;
    rec.p = r.at(t);

    return true;
}



#endif
