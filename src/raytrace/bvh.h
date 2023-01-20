#ifndef BVH_H
#define BVH_H
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

#include "rtweekend.h"

#include "hittable.h"
#include "hittable_list.h"

#include <algorithm>

class bvh_node : public hittable {
public:
  bvh_node();

  bvh_node(const hittable_list &list, double time0, double time1)
      : bvh_node(list.objects, 0, list.objects.size(), time0, time1) {}

  bvh_node(const std::vector<shared_ptr<hittable>> &src_objects, size_t start,
           size_t end, double time0, double time1);

  virtual bool hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const override;

  virtual bool bounding_box(double time0, double time1,
                            aabb &output_box) const override;

public:
  shared_ptr<hittable> left;
  shared_ptr<hittable> right;
  aabb box;
};

inline bool box_compare(const shared_ptr<hittable> a,
                        const shared_ptr<hittable> b, int axis) {
  aabb box_a;
  aabb box_b;

  if (!a->bounding_box(0, 0, box_a) || !b->bounding_box(0, 0, box_b))
    std::cerr << "No bounding box in bvh_node constructor.\n";

  return box_a.min().e[axis] < box_b.min().e[axis];
}

bool box_x_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
  return box_compare(a, b, 0);
}

bool box_y_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
  return box_compare(a, b, 1);
}

bool box_z_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
  return box_compare(a, b, 2);
}

bvh_node::bvh_node(const std::vector<shared_ptr<hittable>> &src_objects,
                   size_t start, size_t end, double time0, double time1) {
  auto objects =
      src_objects; // Create a modifiable array of the source scene objects

  size_t object_span = end - start;

  if (object_span == 1) {
    left = right = objects[start];
  } else if (object_span == 2) {
    left = objects[start];
    right = objects[start + 1];
  } else {
    aabb _box;
    // determine which axis
    for (auto obj : objects) {

      aabb box_obj;
      bool flag = obj->bounding_box(time0, time1, box_obj);
      _box = surrounding_box(_box, box_obj);
    }
    int axis = random_int(0, 2);
    // int axis = _box.longest_axis();
    auto comparator = (axis == 0)   ? box_x_compare
                      : (axis == 1) ? box_y_compare
                                    : box_z_compare;
    std::sort(objects.begin() + start, objects.begin() + end, comparator);

    if (object_span < 12) {
      auto mid = start + object_span / 2;
      left = make_shared<bvh_node>(objects, start, mid, time0, time1);
      right = make_shared<bvh_node>(objects, mid, end, time0, time1);
    } else {
      // employ SAH algorithm to accelerate the BVH
      auto l = (float)object_span;
      float nums[] = {1.0 / 10, 2.0 / 10, 3.0 / 10, 4.0 / 10, 5.0 / 10,
                      6.0 / 10, 7.0 / 10, 8.0 / 10, 9.0 / 10};
      for (int i = 0; i < 9; i++) {
        nums[i] *= l;
      }
      double minCost = std::numeric_limits<double>::max();
      auto beginning = objects.begin();
      auto ending = objects.end();
      int bestChoice = 0;
      int bestIndex = 0;

      for (int i = 0; i < 9; i++) {
        auto middling = objects.begin() + (int)nums[i];
        // compute the size(area) of the box composed by the objects
        auto leftshapes =
            std::vector<shared_ptr<hittable>>(beginning, middling);
        auto rightshapes = std::vector<shared_ptr<hittable>>(middling, ending);
        aabb box_left;
        for (auto obj : leftshapes) {
          aabb left_obj;
          bool flag_left = obj->bounding_box(time0, time1, left_obj);
        }
        double left_heuristic = box_left.area();
        aabb box_right;
        for (auto obj : rightshapes) {
          aabb right_obj;
          bool flag_right = obj->bounding_box(time0, time1, right_obj);
          box_right = surrounding_box(right_obj, box_right);
        }
        double right_heuristic = box_right.area();
        double cost = 2.f + (left_heuristic * leftshapes.size() +
                             right_heuristic * rightshapes.size()) /
                                _box.area();

        if (cost < minCost) {
          bestChoice = (int)nums[i];
          minCost = cost;
          bestIndex = i;
        }
      }

      // std::cerr << std::endl;
      auto middling = objects.begin() + (int)nums[bestIndex];
      // compute the size(area) of the box composed by the objects
      auto leftshapes = std::vector<shared_ptr<hittable>>(beginning, middling);
      auto rightshapes = std::vector<shared_ptr<hittable>>(middling, ending);
      auto mid = start + bestChoice;
      left = make_shared<bvh_node>(leftshapes, start, mid, time0, time1);
      right = make_shared<bvh_node>(rightshapes, start, end - bestChoice, time0,
                                    time1);
    }

  }

  aabb box_left, box_right;

  if (!left->bounding_box(time0, time1, box_left) ||
      !right->bounding_box(time0, time1, box_right))
    std::cerr << "No bounding box in bvh_node constructor.\n";

  box = surrounding_box(box_left, box_right);
}

bool bvh_node::hit(const ray &r, double t_min, double t_max,
                   hit_record &rec) const {
  if (!box.hit(r, t_min, t_max))
    return false;

  bool hit_left = left->hit(r, t_min, t_max, rec);
  bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);

  return hit_left || hit_right;
}

bool bvh_node::bounding_box(double time0, double time1,
                            aabb &output_box) const {
  output_box = box;
  return true;
}

#endif