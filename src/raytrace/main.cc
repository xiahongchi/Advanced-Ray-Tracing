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

#include "aarect.h"
#include "box.h"
#include "bvh.h"
#include "camera.h"
#include "color.h"
#include "hittable_list.h"
#include "list_merge.h"
#include "material.h"
#include "mesh.h"
#include "planes.h"
#include "sphere.h"
#include "texture.h"
#include "triangle.h"
#include "vec3.h"
#include "vertices.h"
#include <iostream>
#include <pthread.h>
#include <vector>

typedef struct task_struct {
  int samples_per_pixel, image_height, image_width;
  std::list<pixel> *color_buffer;
  hittable *world;
  shared_ptr<hittable> lights;
  color background;
  camera *cam;
  int no;
  int *pixel_pool;
  double prob_to_stop;
} task_struct;

pthread_mutex_t print_mutex;
pthread_mutex_t pixel_mutex;
int finished_pixels = 0;
auto lights = make_shared<hittable_list>(true);

color ray_color(const ray &r, const color &background, const hittable &world,
                shared_ptr<hittable> lights, double prob_to_stop) {
  hit_record rec;

  // If we've exceeded the ray bounce limit, no more light is gathered.
  if (random_double() < prob_to_stop)
    return color(0, 0, 0);

  // If the ray hits nothing, return the background color.
  if (!world.hit(r, 0.001, infinity, rec))
    return background / (1 - prob_to_stop);

  scatter_record srec;
  color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);

  if (!rec.mat_ptr->scatter(r, rec, srec))
    return emitted / (1 - prob_to_stop);

  if (srec.is_specular) {
    return (srec.attenuation * ray_color(srec.specular_ray, background, world,
                                         lights, prob_to_stop)) /
           (1 - prob_to_stop);
  }

  auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
  mixture_pdf p(light_ptr, srec.pdf_ptr);
  ray scattered = ray(rec.p, p.generate(), r.time());
  auto pdf_val = p.value(scattered.direction());

  return (emitted +
          srec.attenuation * rec.mat_ptr->scattering_pdf(r, rec, scattered) *
              ray_color(scattered, background, world, lights, prob_to_stop) /
              pdf_val) /
         (1 - prob_to_stop);
}

bool in_region(int x,int y)
{
  if(x>50&&x<350&&y<230&&y>0)
  {
    return true;
  }
  else return false;
}

void *rt_handler(void *task) {

  task_struct *thread_task = (task_struct *)task;

  while (true) {
    pthread_mutex_lock(&pixel_mutex);
    int pixel_loc = *thread_task->pixel_pool;
    if (pixel_loc == 0) {
      pthread_mutex_unlock(&pixel_mutex);
      break;
    }
    *thread_task->pixel_pool = *thread_task->pixel_pool - 1;
    pthread_mutex_unlock(&pixel_mutex);

    int y = (pixel_loc - 1) / thread_task->image_width;
    int x = thread_task->image_width - 1 -
            ((pixel_loc - 1) % thread_task->image_width);
    
    int samples = thread_task->samples_per_pixel;
    if(in_region(x,y))
    {
      samples*=1;
    }


    color pixel_color(0, 0, 0);
    for (int s = 0; s < samples; ++s) {
      auto u = (x + random_double()) / (thread_task->image_width - 1);
      auto v = (y + random_double()) / (thread_task->image_height - 1);
      ray r = thread_task->cam->get_ray(u, v);
      pixel_color +=
          ray_color(r, thread_task->background, *(thread_task->world), lights,
                    thread_task->prob_to_stop);
    }
    write_color(thread_task->color_buffer, pixel_color,
                samples, y, x);
    pthread_mutex_lock(&print_mutex);
    finished_pixels++;
    if (finished_pixels % 1000 == 0)
      std::cerr << "\r Remaining pixels: "
                << thread_task->image_width * thread_task->image_height -
                       finished_pixels
                << "   ";
    pthread_mutex_unlock(&print_mutex);
  }

  free(task);
  return NULL;
}
hittable_list sjtu_world() {
  hittable_list bvh_maker;
  hittable_list objects;

  // externals

  auto emat = make_shared<lambertian>(
      make_shared<image_texture>("/home/yevzwming/code/Raytracing/"
                                 "tra/src/raytrace/star1.jpg"));
  auto sjtu = make_shared<lambertian>(
      make_shared<image_texture>("/home/yevzwming/code/Raytracing/"
                                 "tra/src/raytrace/night.jpg"));
  auto mercury = make_shared<lambertian>(
      make_shared<image_texture>("/home/yevzwming/code/Raytracing/"
                                 "tra/src/raytrace/Mercury.jpg"));
  auto moon = make_shared<lambertian>(
      make_shared<image_texture>("/home/yevzwming/code/Raytracing/"
                                 "tra/src/raytrace/surface.jpg"));
  auto sjtu1 = make_shared<lambertian>(make_shared<image_texture>(
      "/home/yevzwming/code/Raytracing/tra/src/raytrace/1.png"));
  auto sjtu2 = make_shared<lambertian>(make_shared<image_texture>(
      "/home/yevzwming/code/Raytracing/tra/src/raytrace/2.jpg"));
  auto sjtu3 = make_shared<lambertian>(make_shared<image_texture>(
      "/home/yevzwming/code/Raytracing/tra/src/raytrace/3.jpg"));
  auto sjtu4 = make_shared<lambertian>(make_shared<image_texture>(
      "/home/yevzwming/code/Raytracing/tra/src/raytrace/baihe.jpg"));
  auto sjtu5 = make_shared<lambertian>(make_shared<image_texture>(
      "/home/yevzwming/code/Raytracing/tra/src/raytrace/5.png"));
  auto sjtu6 = make_shared<lambertian>(make_shared<image_texture>(
      "/home/yevzwming/code/Raytracing/tra/src/raytrace/6.png"));
  auto white = make_shared<lambertian>(color(.73, .73, .73));
  auto light = make_shared<diffuse_light>(color(15, 15, 15));
  auto light2 = make_shared<diffuse_light>(color(6, 6, 6));
  auto green = make_shared<lambertian>(color(.12, .45, .15));
  auto red = make_shared<metal>(color(.65, .05, .05), 1.5);
  auto blue = make_shared<metal>(color(.00, .25, .60), 1.5);
  auto metaltest = make_shared<metal>(color(0.8, 0.2, 0.3), 1.0);
  auto pink = make_shared<diffuse_light>(color(.95, .74, .78));
  auto glass = make_shared<dielectric>(1.5);
  objects.add(make_shared<xz_rect>(0, 1000, 0, 800, 0, glass));
  // bvh_maker.add(make_shared<xz_rect>(0, 1000, 0, 800, 600, white));
  // bvh_maker.add(make_shared<yz_rect>(0, 555, 0, 555, 0, glass));
  objects.add(make_shared<flip_face>(
      make_shared<xz_rect>(400, 600, 300, 500, 599, light)));
  objects.add(make_shared<xy_rect>(0, 1000, 0, 600, 800, sjtu));
  // bvh_maker.add(make_shared<xy_rect>(0, 1000, 0, 600, 800, light2));
  // bvh_maker.add(make_shared<yz_rect>(0, 600, 0, 800, 1000, red));
  objects.add(make_shared<yz_rect>(0, 600, 0, 800, 0, glass));

  // light
  objects.add(make_shared<sphere>(point3(500, 800, 400), 100, light));
  objects.add(make_shared<sphere>(point3(100, 400, 50), 50, light));
  objects.add(
      make_shared<sphere>(point3(525, 175, 200), 20,
                          make_shared<diffuse_light>(color(.99, .99, .50))));
  // objects.add(make_shared<mesh>("/home/yevzwming/code/Raytracing/tra/src/raytrace/tri.obj",1,50,vec3(350,200,100),vec3(0,0,0),red));
  objects.add(make_shared<sphere>(point3(1000, 100, 0), 50, light));

  // ground light
  objects.add(
      make_shared<sphere>(point3(100, -30, 600), 80,
                          make_shared<diffuse_light>(make_shared<image_texture>(
                              "/home/yevzwming/code/Raytracing/"
                              "tra/src/raytrace/Mercury.jpg"))));
  objects.add(
      make_shared<sphere>(point3(100, -30, 100), 60,
                          make_shared<diffuse_light>(make_shared<image_texture>(
                              "/home/yevzwming/code/Raytracing/"
                              "tra/src/raytrace/Mercury.jpg"))));
  objects.add(
      make_shared<sphere>(point3(600, -50, 170), 100,
                          make_shared<diffuse_light>(make_shared<image_texture>(
                              "/home/yevzwming/code/Raytracing/"
                              "tra/src/raytrace/Mercury.jpg"))));
  objects.add(
      make_shared<sphere>(point3(800, -50, 600), 80,
                          make_shared<diffuse_light>(make_shared<image_texture>(
                              "/home/yevzwming/code/Raytracing/"
                              "tra/src/raytrace/Mercury.jpg"))));
  objects.add(
      make_shared<sphere>(point3(100, 400, 550), 80,
                          make_shared<diffuse_light>(make_shared<image_texture>(
                              "/home/yevzwming/code/Raytracing/"
                              "tra/src/raytrace/surface.jpg"))));

  // bvh_maker.add(make_shared<mesh>(
  //     "/home/yevzwming/code/Raytracing/tracing-for-you/src/raytrace/xh.obj",
  //     2, 15, vec3(625, 200, 325), vec3(0, 240, 0), red));
  // bvh_maker.add(make_shared<sphere>(point3(600, 80, 650), 80, metaltest));
  // bvh_maker.add(make_shared<sphere>(point3(700, 200, 550), 70, emat));

  // 多面体交大

  objects.add(make_shared<xy_rect>(500, 700, 0, 200, 450, sjtu4));
  objects.add(make_shared<xy_rect>(500, 700, 0, 200, 250, sjtu4));
  objects.add(make_shared<xz_rect>(500, 700, 250, 450, 0, sjtu4));
  objects.add(make_shared<xz_rect>(500, 700, 250, 450, 200, sjtu4));
  objects.add(make_shared<yz_rect>(0, 200, 250, 450, 500, sjtu4));
  objects.add(make_shared<yz_rect>(0, 200, 250, 450, 700, sjtu4));
  // bvh_maker.add(make_shared<sphere>(point3(250,250,450),300,glass));
  objects.add(make_shared<box>(point3(475, 0, 225), point3(725, 225, 475),
                               make_shared<dielectric>(1.15)));

  // sjtu球
  hittable_list spheres;
  int biasx = 110, biasy = 20;
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 5; j++) {
      int x = i * 20 + biasx;
      int y = j * 20 + biasy;
      int z = 150;
      switch (j) {
      case 0: {
        if (i == 0 || i == 1 || i == 2  || i == 5 || i == 9 ||
            i == 10 || i == 12 || i == 13 || i == 14) {
          spheres.add(make_shared<sphere>(point3(x, y, 200), 6, pink));
          spheres.add(
              make_shared<sphere>(point3(x + 5, y + 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x - 5, y - 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x + 5, y - 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x - 5, y + 5, z + 8), 3, mercury));
        }
        break;
      }
      case 1: {
        if (i == 0 || i == 2 || i == 5 || i == 9 || i == 12) {
          spheres.add(make_shared<sphere>(point3(x, y, 200), 7, pink));
          spheres.add(make_shared<sphere>(point3(x + 5, y, z + 8), 3,
          mercury)); spheres.add(make_shared<sphere>(point3(x - 5, y, z + 8),
          3, mercury)); spheres.add(make_shared<sphere>(point3(x, y - 5, z +
          8), 3, mercury)); spheres.add(make_shared<sphere>(point3(x - 5, y,
          z + 8), 3, mercury));
        }
        break;
      }
      case 2: {
        if (i == 0 || i == 2 || i == 5 || i == 9 || i == 12 || i == 13 ||
            i == 14) {
          spheres.add(make_shared<sphere>(point3(x, y, 200), 7,pink));
          spheres.add(
              make_shared<sphere>(point3(x + 5, y + 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x - 5, y - 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x + 5, y - 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x - 5, y + 5, z + 8), 3, mercury));
        }
        break;
      }
      case 3: {
        if (i == 0 || i == 2 || i == 5 || i == 9 || i == 14) {
          spheres.add(make_shared<sphere>(point3(x, y, 200), 7, pink));
          spheres.add(make_shared<sphere>(point3(x + 5, y, z + 8), 3,
          mercury)); spheres.add(make_shared<sphere>(point3(x - 5, y, z + 8),
          3, mercury)); spheres.add(make_shared<sphere>(point3(x, y - 5, z +
          8), 3, mercury)); spheres.add(make_shared<sphere>(point3(x, y + 5,
          z + 8), 3, mercury));
        }
        break;
      }
      case 4: {
        if (i == 0 || i == 2 || i == 4 || i == 5 || i == 6 || i == 8 ||
            i == 9 || i == 10 || i == 12 || i == 13 || i == 14) {
          spheres.add(make_shared<sphere>(point3(x, y, 200), 7,pink));
          spheres.add(
              make_shared<sphere>(point3(x + 5, y + 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x - 5, y - 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x + 5, y - 5, z + 8), 3, mercury));
          spheres.add(
              make_shared<sphere>(point3(x - 5, y + 5, z + 8), 3, mercury));
        }
        break;
      }
      }
    }
  }
  objects.add(make_shared<bvh_node>(spheres, 0, 1));

  objects.add(make_shared<sphere>(point3(300, 200, 300), 80, glass));
  // objects.add(make_shared<sphere>(point3(600, 275, 350), 50, glass));

  //三棱锥
  hittable_list trian;
  auto v1 = vec3(0, 0, 0) + point3(550, 250, 350);
  auto v2 = vec3(80, 0, 60) + point3(550, 250, 350);
  auto v3 = vec3(-20, 0, 80) + point3(550, 250, 350);
  auto v4 = vec3(80, 150, 60) + point3(550, 250, 350);
  trian.add(make_shared<triangle>(v1, v2, v3, glass));
  trian.add(make_shared<triangle>(v1, v2, v4, glass));
  trian.add(make_shared<triangle>(v1, v3, v4, glass));
  trian.add(make_shared<triangle>(v2, v3, v4, glass));
  bvh_maker.add(make_shared<bvh_node>(trian, 0, 1));

    // shadow
      // objects.add(make_shared<sphere>(point3(100, 400, 50), 50, light));
      objects.add(make_shared<sphere>(point3(100,240,270),50,white));
      objects.add(make_shared<sphere>(point3(100,300,200),40,emat));


  bvh_maker.add(make_shared<mesh>("/home/yevzwming/code/Raytracing/tra/src/raytrace/xh.obj",2,15,vec3(720,350,350),vec3(0,240,0),blue));
  bvh_maker.add(make_shared<bvh_node>(objects, 0, 1));





  return bvh_maker;
}

hittable_list cornell_box() {
  hittable_list objects;

  auto red = make_shared<lambertian>(color(.65, .05, .05));
  auto white = make_shared<lambertian>(color(.73, .73, .73));
  auto green = make_shared<lambertian>(color(.12, .45, .15));
  auto light = make_shared<diffuse_light>(color(15, 15, 15));

  objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
  objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
  objects.add(make_shared<flip_face>(
      make_shared<xz_rect>(213, 343, 227, 332, 554, light)));
  objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
  objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
  objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

  //   point3 lookfrom(478, 278, -700);
  //   point3 lookat(278, 278, 0);
  //   vec3 vup(0, 1, 0);
  //   auto dist_to_focus = 10.0;
  //   auto aperture = 0.0;
  //   auto vfov = 40.0;
  //   auto time0 = 0.0;
  //   auto time1 = 1.0;
  shared_ptr<material> aluminum =
      make_shared<metal>(color(0.8, 0.85, 0.88), 0.0);
  // shared_ptr<hittable> box1 = make_shared<box>(point3(0,0,0),
  // point3(165,330,165), aluminum); box1 = make_shared<rotate_y>(box1, 15);
  // box1 = make_shared<translate>(box1, vec3(265,0,295));
  // objects.add(box1);

  auto glass = make_shared<dielectric>(1.5);
  objects.add(make_shared<sphere>(point3(190, 90, 190), 90, glass));

  hittable_list trian;
  auto v1 = vec3(0, 0, 0) + point3(100, 300, 100);
  auto v2 = vec3(100, 0, 130) + point3(100, 300, 100);
  auto v3 = vec3(10, 0, 140) + point3(100, 300, 100);
  auto v4 = vec3(120, 200, 140) + point3(100, 300, 100);
  trian.add(make_shared<triangle>(v1, v2, v3, glass));
  trian.add(make_shared<triangle>(v1, v2, v4, glass));
  trian.add(make_shared<triangle>(v1, v3, v4, glass));
  trian.add(make_shared<triangle>(v2, v3, v4, glass));
  objects.add(make_shared<bvh_node>(trian, 0, 1));
  // construct a obj file
  // v 1.000000 1.000000 -1.000000
  // v 1.000000 -1.000000 -1.000000
  // v 1.000000 1.000000 1.000000
  // v 1.000000 -1.000000 1.000000
  // v -1.000000 1.000000 -1.000000
  // v -1.000000 -1.000000 -1.000000
  // v -1.000000 1.000000 1.000000
  // v -1.000000 -1.000000 1.000000

  // refer to mesh.h
  objects.add(make_shared<mesh>(
      "/home/yevzwming/code/Raytracing/tra/src/raytrace/xh.obj", 2, 15,
      vec3(600, 200, 350), vec3(0, 240, 0), red));

  // std::vector<point3> square_points;
  // square_points.push_back(point3(1.000000, 1.000000, -1.000000));
  // square_points.push_back(point3(1.000000, -1.000000, -1.000000));
  // square_points.push_back(point3(1.000000, 1.000000, 1.000000));
  // square_points.push_back(point3(1.000000, -1.000000, 1.000000));
  // square_points.push_back(point3(-1.000000, 1.000000, -1.000000));
  // square_points.push_back(point3(-1.000000, -1.000000, -1.000000));
  // square_points.push_back(point3(-1.000000, 1.000000, 1.000000));
  // square_points.push_back(point3(-1.000000, -1.000000, 1.000000));
  // auto square_vertices = vertices(square_points);
  // square_vertices.scale(50);
  // square_vertices.translate(vec3(365,100,395));

  // f 1/1/1 5/2/1 7/3/1 3/4/1
  // usemtl Material.001
  // f 4/5/2 3/4/2 7/6/2 8/7/2
  // f 8/8/3 7/9/3 5/10/3 6/11/3
  // f 6/12/4 2/13/4 4/5/4 8/14/4
  // f 2/13/5 1/1/5 3/4/5 4/5/5
  // f 6/11/6 5/10/6 1/1/6 2/13/6

  // 0 4 6 2
  // 3 2 6 7
  // 7 6 4 5
  // 5 1 3 7
  // 1 0 2 3
  // 5 4 0 1

  // std::vector<std::vector<int>> planes_nodes_nums;
  // planes_nodes_nums.push_back(std::vector<int>({0, 4, 6, 2}));
  // planes_nodes_nums.push_back(std::vector<int>({3, 2, 6, 7}));
  // planes_nodes_nums.push_back(std::vector<int>({7, 6, 4, 5}));
  // planes_nodes_nums.push_back(std::vector<int>({5, 1, 3, 7}));
  // planes_nodes_nums.push_back(std::vector<int>({1, 0, 2, 3}));
  // planes_nodes_nums.push_back(std::vector<int>({5, 4, 0, 1}));

  // auto square_planes = make_shared<planes>(planes_nodes_nums,
  // square_vertices, green); objects.add(square_planes);

  // std::vector<point3> square_points_second;
  // square_points_second.push_back(point3(0.500000, 1.000000, -0.500000));
  // square_points_second.push_back(point3(1.000000, -1.000000, -1.000000));
  // square_points_second.push_back(point3(0.500000, 1.000000, 0.500000));
  // square_points_second.push_back(point3(1.000000, -1.000000, 1.000000));
  // square_points_second.push_back(point3(-0.500000, 1.000000, -0.500000));
  // square_points_second.push_back(point3(-1.000000, -1.000000, -1.000000));
  // square_points_second.push_back(point3(-0.500000, 1.000000, 0.500000));
  // square_points_second.push_back(point3(-1.000000, -1.000000, 1.000000));

  // auto square_vertices_second = vertices(square_points_second);
  // square_vertices_second.scale(75);
  // square_vertices_second.translate(vec3(400,500,300));

  // auto second_planes = make_shared<planes>(planes_nodes_nums,
  // square_vertices_second, glass); objects.add(second_planes);

  return objects;
}

int main() {
  // Parallel
  const int nthreads = 16;

  // Image

  const auto aspect_ratio = 16.0 / 9.0;
  const int image_width = 800;
  const int image_height = static_cast<int>(image_width / aspect_ratio);
  const int samples_per_pixel = 10000;
  // const int max_depth = 10;
  const double prob_to_stop = 0.05;

  int pixel_pool = image_width * image_height;

  // World
  // auto lights = make_shared<hittable_list>();
  auto world = sjtu_world();
  color background(0, 0, 0);

  lights->add(
      make_shared<xz_rect>(400, 600, 300, 500, 599, shared_ptr<material>()));
  lights->add(
      make_shared<sphere>(point3(500, 800, 400), 100, shared_ptr<material>()));
  lights->add(
      make_shared<sphere>(point3(100, 400, 50), 50, shared_ptr<material>()));
  lights->add(
      make_shared<sphere>(point3(525, 175, 20), 20, shared_ptr<material>()));
  lights->add(
      make_shared<sphere>(point3(1000, 100, 0), 50, shared_ptr<material>()));

  // Camera

  point3 lookfrom(540, 200, -400);
  point3 lookat(500, 200, 0);
  vec3 vup(0, 1, 0);
  auto dist_to_focus = 10.0;
  auto aperture = 0.0;
  auto vfov = 40.0;
  auto time0 = 0.0;
  auto time1 = 1.0;

  camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus,
             time0, time1);

  // Render

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  // MultiThread accelerate
  std::list<pixel> *lists = new std::list<pixel>[nthreads];
  pthread_t *rt_threads = (pthread_t *)malloc(nthreads * sizeof(pthread_t));
  for (int nt = 0; nt < nthreads; nt++) {

    task_struct *task = (task_struct *)malloc(sizeof(task_struct));
    task->image_width = image_width;
    task->image_height = image_height;
    task->color_buffer = &lists[nt];

    task->prob_to_stop = prob_to_stop;
    task->samples_per_pixel = samples_per_pixel;
    task->cam = &cam;
    // task->lights = lights;
    task->background = background;
    task->world = &world;
    task->no = nt;
    task->pixel_pool = &pixel_pool;

    if (pthread_create(&rt_threads[nt], NULL, rt_handler, task)) {
      fprintf(stderr, "Error creating thread\n");
      return 1;
    }
  }

  for (int nt = 0; nt < nthreads; nt++) {
    pthread_join(rt_threads[nt], NULL);
  }

  int remain_list = nthreads;
  while (remain_list > 1) {
    int step = nthreads / remain_list;
    for (int i = 0; i < nthreads; i += (2 * step)) {
      if (i + step < nthreads)
        merge_list(lists[i], lists[i + step]);
    }
    remain_list /= 2;
  }

  for (auto c : lists[0]) {
    std::cout << c.r << ' ' << c.g << ' ' << c.b << '\n';
  }

  std::cerr << "\nDone.\n";
}
