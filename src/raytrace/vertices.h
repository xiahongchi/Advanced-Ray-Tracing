#ifndef VERTICES_H
#define VERTICES_H

#include "rtweekend.h"
#include "vec3.h"
#include "hittable_list.h"
#include <vector>

class vertices{
    public:
        vertices(std::vector<point3>& _points){
            points = _points;
        }

        void scale(double size){
            for(int i = 0; i < points.size(); i++){
                points[i] *= size;
            }
        }

        void translate(point3 t){
            for(int i = 0; i < points.size(); i++){
                points[i] += t;
            }
        }
        
        // rotate counter-clockwise
        void rotate_x(double angle){
            auto radians = degrees_to_radians(angle);
            auto sin_theta = sin(radians);
            auto cos_theta = cos(radians);
           
            for(int i = 0; i < points.size(); i++){
                auto y = points[i].e[1];
                auto z = points[i].e[2];
                points[i].e[1] = cos_theta*y - sin_theta*z;
                points[i].e[2] = sin_theta*y + cos_theta*z;
            }
        }

        void rotate_y(double angle){
            auto radians = degrees_to_radians(angle);
            auto sin_theta = sin(radians);
            auto cos_theta = cos(radians);
            
            for(int i = 0; i < points.size(); i++){
                auto x = points[i].e[0];
                auto z = points[i].e[2];
                points[i].e[0] = cos_theta*x + sin_theta*z;
                points[i].e[2] = -sin_theta*x + cos_theta*z;
            }
        }

        void rotate_z(double angle){
            auto radians = degrees_to_radians(angle);
            auto sin_theta = sin(radians);
            auto cos_theta = cos(radians);
            
            for(int i = 0; i < points.size(); i++){
                auto x = points[i].e[0];
                auto y = points[i].e[1];
                points[i].e[0] = cos_theta*x - sin_theta*y;
                points[i].e[1] = sin_theta*x + cos_theta*y;
            }
        }

        void rotate(vec3 angles){
            rotate_x(angles.e[0]);
            rotate_y(angles.e[1]);
            rotate_z(angles.e[2]);
        }

        

        void extract(std::vector<int>& node_nums, std::vector<point3>& vps){
            for(auto p: node_nums){
                if(0 <= p < points.size()){
                    vps.push_back(points[p]);
                }
                else{
                    std::cerr << "invalid point subscript\n";
                }
            }
        }

    public:
        std::vector<point3> points;

};




#endif