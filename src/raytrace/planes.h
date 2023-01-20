#ifndef PLANES_H
#define PLANES_H

#include "rtweekend.h"
#include "vec3.h"
#include "hittable_list.h"
#include "vertices.h"
#include <vector>
#include "bvh.h"

class plane : public hittable{
    public: 

        plane() {}
        plane(std::vector<point3>& _plane_nodes, shared_ptr<material> mat): mp(mat), plane_nodes(_plane_nodes){
            if(plane_nodes.size() < 3){
                std::cerr << "invalid plane\n";
            }
            for(int i = 0; i < plane_nodes.size(); i++){
                plane_edges.push_back(plane_nodes[(i+1) % plane_nodes.size()] - plane_nodes[i]);
            }
            _xmin = plane_nodes[0].x();
            _ymin = plane_nodes[0].y();
            _zmin = plane_nodes[0].z();
            _xmax = plane_nodes[0].x();
            _ymax = plane_nodes[0].y();
            _zmax = plane_nodes[0].z();
             
            for(auto p : plane_nodes){
                _xmin = fmin(_xmin, p.x());
                _ymin = fmin(_ymin, p.y());
                _zmin = fmin(_zmin, p.z());
                _xmax = fmax(_xmax, p.x());
                _ymax = fmax(_ymax, p.y());
                _zmax = fmax(_zmax, p.z());
            }
        }

        virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override {
            auto normal = unit_vector(cross(plane_edges[0], plane_edges[1]));

            if(fabs(dot(normal, unit_vector(r.direction()))) < 0.0001)
                return false;

            auto t = dot(plane_nodes[0] - r.origin(), normal) / dot(r.direction(), normal);
            if(t < t_min || t > t_max)
                return false;

            auto p = r.at(t);
            auto base = cross(plane_nodes[0] - p, plane_edges[0]);

            for(int i = 1; i < plane_edges.size(); i++){
                if(dot(base, cross(plane_nodes[i] - p, plane_edges[i])) < 0)
                    return false;
            }

            rec.u = 0.5;
            rec.v = 0.5;
            rec.t = t;
            auto outward_normal = normal;
            rec.set_face_normal(r, outward_normal);
            rec.mat_ptr = mp;
            rec.p = p;

            return true;
        }

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
            // The bounding box must have non-zero width in each dimension, so pad the Y
            // dimension a small amount.
            output_box = aabb(point3(_xmin-0.0001,_ymin-0.0001,_zmin-0.0001), point3(_xmax+0.0001, _ymax+0.0001, _zmax+0.0001));
            return true;
        }

        double xmin(){return _xmin;}
        double ymin(){return _ymin;}
        double zmin(){return _zmin;}
        double xmax(){return _xmax;}
        double ymax(){return _ymax;}
        double zmax(){return _zmax;}

    public:
        std::vector<point3> plane_nodes;
        std::vector<point3> plane_edges;
        double _xmin, _ymin, _zmin, _xmax, _ymax, _zmax;
        shared_ptr<material> mp;
};

class planes: public hittable{

    public:
        planes() {}
        planes(std::list<std::vector<int>> planes_nodes_num, vertices& _vertices, shared_ptr<material> ptr);

        virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
            return sides.bounding_box(time0, time1, output_box);
        }

    public:
        hittable_list sides;
        shared_ptr<bvh_node> node;

};

planes::planes(std::list<std::vector<int>> planes_nodes_nums, vertices& _vertices, shared_ptr<material> ptr) {
    
    std::vector<point3> vps_init;
    _vertices.extract(*planes_nodes_nums.begin(), vps_init);
    for(auto plane_node_nums: planes_nodes_nums){
        std::vector<point3> vps;
        _vertices.extract(plane_node_nums, vps);
        sides.add(make_shared<plane>(vps, ptr));
    }
    std::cerr << "make bvh_node\n";
    node = make_shared<bvh_node>(sides,0,1);
    std::cerr << "after make bvh_node\n";

}

bool planes::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    return node->hit(r, t_min, t_max, rec);
    // return sides.hit(r, t_min, t_max, rec);
}

#endif