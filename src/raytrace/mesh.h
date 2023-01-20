#ifndef MESH_H
#define MESH_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright
// and related and neighboring rights to this software to the public domain
// worldwide. This software is distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public
// Domain Dedication along with this software. If not, see
// <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include "aarect.h"
#include "box.h"
#include "bvh.h"
#include "camera.h"
#include "color.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "planes.h"
#include "rtweekend.h"
#include "vertices.h"
#include <list>
#include <vector>

#include "sphere.h"
#include "triangle.h"

#include "stdio.h"
#include "vec3.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

class mesh : public hittable {
public:
  mesh() {}
  mesh(const char *filename, int flag, int scale, vec3 translate, vec3 rotate,
       shared_ptr<material> mat)
      : mp(mat) {
    string strTemp;
    ifstream infile;
    infile.open(filename);
    if (!infile.is_open())
      cerr << "Error occurred!\n";
    string sline, s0;
    while (getline(infile, sline)) {
      if (sline[0] == 'v' && sline[1] == ' ') {
        istringstream iss(sline.substr(1));
        point3 pos;
        iss >> pos[0] >> pos[1] >> pos[2];

        square_points.push_back(pos);
        // cerr << "pos:"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<"\n";

      } else if (sline[0] == 'f') {
        istringstream iss(sline.substr(1)); // the one for store the data
        istringstream isss(
            sline.substr(1)); // the one for judging when to finish
        std::vector<int> firstIndex;
        int i, j, k;
        char c;
        int cnt = 0;
        // flag means that case 3: a/b/c ||||||||||||case 2: a//b
        switch (flag) {
        case 3:
          while (isss >> strTemp) {
            iss >> i;
            iss >> c;
            iss >> j;
            iss >> c;
            iss >> k;
            firstIndex.push_back(i - 1);
            cnt++;
          }
          break;
        case 2:
          while (isss >> strTemp) {
            iss >> i;
            iss >> c;
            // iss >> j;
            iss >> c;
            iss >> k;
            firstIndex.push_back(i - 1);
            cnt++;
          }
          break;
        case 1:
          while (isss >> strTemp) {
            iss >> i;
            // iss >> c;
            // // iss >> j;
            // iss >> c;
            // iss >> k;
            firstIndex.push_back(i - 1);
            cnt++;
          }
        }

        // for(auto index: firstIndex){
        //   cerr<<index<<' ';
        // }
        // cerr<<'\n';
        planes_nodes_nums.push_back(firstIndex);
      }
    }
    auto square_vertices = vertices(square_points);
    square_vertices.rotate(rotate);
    square_vertices.scale(scale);
    square_vertices.translate(translate);
    std::cerr << "start planes generated.\n";
    auto square_planes =
        make_shared<planes>(planes_nodes_nums, square_vertices, mat);
    std::cerr << "after planes generated.\n";
    objs.add(square_planes);

    // compute max and min
    _xmin = square_points[0].x();
    _ymin = square_points[0].y();
    _zmin = square_points[0].z();
    _xmax = square_points[0].x();
    _ymax = square_points[0].y();
    _zmax = square_points[0].z();

    for (auto p : square_points) {
      _xmin = fmin(_xmin, p.x());
      _ymin = fmin(_ymin, p.y());
      _zmin = fmin(_zmin, p.z());
      _xmax = fmax(_xmax, p.x());
      _ymax = fmax(_ymax, p.y());
      _zmax = fmax(_zmax, p.z());
    }

    node = make_shared<bvh_node>(objs, 0, 1);
  };
  virtual bool hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const override;
  virtual bool bounding_box(double time0, double time1,
                            aabb &output_box) const override {
    output_box = aabb(point3(_xmin - 0.0001, _ymin - 0.0001, _zmin - 0.0001),
                      point3(_xmax + 0.0001, _ymax + 0.0001, _zmax + 0.0001));
    return true;
  }

public:
  shared_ptr<material> mp;
  shared_ptr<bvh_node> node;
  hittable_list objs;
  std::vector<point3> square_points;
  std::list<std::vector<int>> planes_nodes_nums;
  double _xmin, _ymin, _zmin, _xmax, _ymax, _zmax;
};

bool mesh::hit(const ray &r, double t_min, double t_max,
               hit_record &rec) const {
  return node->hit(r, t_min, t_max, rec);
}

#endif