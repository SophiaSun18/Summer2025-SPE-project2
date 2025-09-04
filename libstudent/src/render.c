/**
 * Author: Isabel Rosa, isarosa@mit.edu, 
 * Jay Hilton, jhilton@mit.edu, 
 * Krit Boonsiriseth, talkon@mit.edu
 **/

#include <assert.h>
#include <cilk/cilk.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "../../common/render.h"
#include "../include/misc_utils.h"

#define PRINT_MESSAGE 0

typedef struct renderer_state {
  renderer_spec_t r_spec;
  float *img;
} renderer_state_t;

typedef struct {
  float x, y;
} vector_2d;

typedef struct {
  vector_t pos;
  float red, green, blue;
} pre_light;

renderer_state_t* init_renderer(const renderer_spec_t *spec) {
  renderer_state_t *state = (renderer_state_t*)malloc(sizeof(renderer_state_t));
  state->r_spec = *spec;
  int n_pixels = state->r_spec.resolution * state->r_spec.resolution;
  state->img = calloc(3ull * (size_t) n_pixels, sizeof(float));
  assert(state->img != NULL);
  return state;
}

void destroy_renderer(renderer_state_t *state) {
  free(state->img);
  free(state);
}

// Computes the ray from a given origin (usually the eye location) to the pixel (x, y)
// in image coordinates.
ray_t origin_to_pixel(renderer_state_t *state, int x, int y) {
  ray_t viewingRay;

  const float pixel_size = state->r_spec.viewport_size / state->r_spec.resolution;
  
  // Center image frame
  float us = -state->r_spec.resolution / 2 + x;
  float vs = -state->r_spec.resolution / 2 + y;

  viewingRay.origin = state->r_spec.eye;
  viewingRay.dir =
    qsubtract(
      qadd(
        scale(us * pixel_size, state->r_spec.proj_plane_u), 
        scale(vs * pixel_size, state->r_spec.proj_plane_v)
      ),
      viewingRay.origin
    );
  viewingRay.dir = scale(1 / qsize(viewingRay.dir), viewingRay.dir);

  return viewingRay;
}

// Determines whether the ray r and the sphere s intersect.
// 
// If the ray and the sphere intersect, writes the distance to the closer intersection
// to `out`, and returns 1. Otherwise, returns 0.
int ray_sphere_intersection(ray_t *r, const sphere_t* s, float *out) {
  vector_t dist = qsubtract(r->origin, s->pos);
  // Uses quadratic formula to compute intersection
  float a = qdot(r->dir, r->dir);
  float b = 2 * qdot(r->dir, dist);
  float c = (float)((double)qdot(dist, dist) - (double)(s->r * s->r));
  float discr = (float)((double)(b * b) - (double)(4 * a * c));

  if (discr >= 0) {
    // Ray hits sphere
    float sqrtdiscr = sqrtf(discr);

    float min_dist;
    if (b >= 0) {
      float sol1 = (float)((double)-b - (double)sqrtdiscr) / (2 * a);
      float sol2 = (float)(2 * (double)c) / ((double)-b - (double)sqrtdiscr);
      min_dist = min(sol1, sol2);
    } else {
      float sol1 = (float)(2 * (double)c) / ((double)-b + (double)sqrtdiscr);
      float sol2 = (float)((double)-b + (double)sqrtdiscr) / (2 * a);
      min_dist = min(sol1, sol2);
    }

    // If new_t > 0 and smaller than original t, we
    // found a new, closer ray-sphere intersection
    if (min_dist > 0) {
      *out = min_dist;
      return 1;
    }
  }

  return 0;
}

// Sorts given spheres by length of the tangent (NOT in-place). 
// Returns a pointer to the sorted spheres. Returned pointer must be freed.
//
// Since the spheres are non-intersecting, this ensures that 
// if sphere S comes before sphere T in this ordering, then 
// sphere S is in front of sphere T in the rendering.
sphere_t* sort(renderer_state_t *state, const sphere_t *spheres, int n_spheres) {
  sphere_t key;
  sphere_t* out = clone(spheres, sizeof(sphere_t) * n_spheres);
  for (int i = 1; i < n_spheres; i++) {
    key = out[i];
    int j = i - 1;

    float d_ke = qdist(key.pos, state->r_spec.eye);
    float r_k = key.r;
    
    while (j >= 0 && 
          (qdist(out[j].pos, state->r_spec.eye) * qdist(out[j].pos, state->r_spec.eye) - out[j].r * out[j].r) > 
          (d_ke * d_ke - r_k * r_k)) {
      out[j + 1] = out[j];
      j = j - 1;
    }
    out[j + 1] = key;
  }

  return out;
}

void set_pixel(renderer_state_t *state, int x, int y, float red, float green, float blue) {
  state->img[(x + y * state->r_spec.resolution) * 3 + 0] = min((float)red, 1.0);
  state->img[(x + y * state->r_spec.resolution) * 3 + 1] = min((float)green, 1.0);
  state->img[(x + y * state->r_spec.resolution) * 3 + 2] = min((float)blue, 1.0);
}

int check_edge(int val, int min_val, int max_val) {
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}

vector_2d project_to_plane(vector_t corner, vector_t e, vector_t u, vector_t v, vector_t w, float pixel_size) {
  vector_t dir = qsubtract(e, corner);
  float k = qdot(e, w) / qdot(dir, w) * -1;
  vector_t p = qadd(e, scale(k, dir));
  vector_2d xy = {qdot(p, u) / pixel_size, qdot(p, v) / pixel_size};

  if (PRINT_MESSAGE) {
    printf("corner x: %f, y: %f, z: %f\n", corner.x, corner.y, corner.z);
    printf("dir x: %f, y: %f, z: %f\n", dir.x, dir.y, dir.z);
    printf("k: %f\n", k);
    printf("p x: %f, y: %f, z: %f\n", p.x, p.y, p.z);
    printf("x: %f, y: %f\n\n", xy.x, xy.y);      
  }

  return xy;
}

vector_t e;

int quick_sort_spheres(const void* a, const void* b) {
  sphere_t* s_a = (sphere_t*) a;
  sphere_t* s_b = (sphere_t*) b;

  float dist_a = qdist(s_a->pos, e) * qdist(s_a->pos, e) - s_a->r * s_a->r;
  float dist_b = qdist(s_b->pos, e) * qdist(s_b->pos, e) - s_b->r * s_b->r;

  if (dist_a < dist_b) return -1;
  if (dist_b < dist_a) return 1;
  return 0;
}

const float* render(renderer_state_t *state, const sphere_t *spheres, int n_spheres) {

  // printf("This is the new render function with %u spheres!\n", n_spheres);

  // sort the spheres first as before
  e = state->r_spec.eye;
  sphere_t* sorted_spheres = malloc(sizeof(sphere_t) * n_spheres);
  memcpy(sorted_spheres, spheres, sizeof(sphere_t) * n_spheres);
  qsort(sorted_spheres, n_spheres, sizeof(sphere_t), quick_sort_spheres);
  // sphere_t* sorted_spheres = sort(state, spheres, n_spheres);
  assert(sorted_spheres != NULL);

  int res = state->r_spec.resolution;
  float pixel_size = state->r_spec.viewport_size / state->r_spec.resolution;
  vector_t u = state->r_spec.proj_plane_u;
  vector_t v = state->r_spec.proj_plane_v;
  vector_t w = qcross(u, v);

  if (PRINT_MESSAGE) {
    printf("e: %f %f %f\n", e.x, e.y, e.z);
    printf("u: %f %f %f\n", u.x, u.y, u.z);
    printf("v: %f %f %f\n", v.x, v.y, v.z);
    printf("w: %f %f %f\n\n", w.x, w.y, w.z);
  }

  // create a vector to keep track of colored pixels
  bool *colored = malloc(res * res * sizeof(bool));
  for (int a = 0; a < res * res; a++)
      colored[a] = false;

  // loop through each sphere
  for (int i = 0; i < n_spheres; i++) {
    sphere_t currentSphere = sorted_spheres[i];
    material_t currentMat = currentSphere.mat;

    vector_t sphereCenter = currentSphere.pos;
    float sphereRadius = currentSphere.r;

    float min_x = sphereCenter.x - sphereRadius, max_x = sphereCenter.x + sphereRadius;
    float min_y = sphereCenter.y - sphereRadius, max_y = sphereCenter.y + sphereRadius;
    float min_z = sphereCenter.z - sphereRadius, max_z = sphereCenter.z + sphereRadius;

    if (PRINT_MESSAGE) {
      printf("\ncenter: %f, %f, %f, radius: %f\n", sphereCenter.x, sphereCenter.y, sphereCenter.z, sphereRadius);
      printf("%u x: %f %f, y: %f %f, z: %f %f\n", i, min_x, max_x, min_y, max_y, min_z, max_z);
    }

    // collect the 8 corners of the cube in space and project to the 2d plane
    vector_2d bound_box[8];
    int count = 0;

    for (int x = 0; x < 2; x++) {
      for (int y = 0; y < 2; y++) {
        for (int z = 0; z < 2; z++) {
          vector_t corner;
          corner.x = (x == 0) ? min_x : max_x;
          corner.y = (y == 0) ? min_y : max_y;
          corner.z = (z == 0) ? min_z : max_z;

          vector_2d xy = project_to_plane(corner, e, u, v, w, pixel_size);
          bound_box[count++] = xy;
        }
      }
    }

    float min_x_2d = bound_box[0].x, max_x_2d = bound_box[0].x;
    float min_y_2d = bound_box[0].y, max_y_2d = bound_box[0].y;

    for (int n = 0; n < 8; n++) {
      min_x_2d = bound_box[n].x < min_x_2d ? bound_box[n].x : min_x_2d;
      max_x_2d = bound_box[n].x > max_x_2d ? bound_box[n].x : max_x_2d;
      min_y_2d = bound_box[n].y < min_y_2d ? bound_box[n].y : min_y_2d;
      max_y_2d = bound_box[n].y > max_y_2d ? bound_box[n].y : max_y_2d;
    }
    if (PRINT_MESSAGE) printf("min_x: %f, max_x: %f, min_y: %f, max_y: %f\n", min_x_2d, max_x_2d, min_y_2d, max_y_2d);
    
    // coordinate conversion, round up and check edge to get the final bounding box range
    int half_size = res / 2;
    int x0 = check_edge((int)floorf(min_x_2d + half_size), 0, res - 1);
    int x1 = check_edge((int)ceilf(max_x_2d + half_size), 0, res - 1);
    int y0 = check_edge((int)floorf(min_y_2d + half_size), 0, res - 1);
    int y1 = check_edge((int)ceilf(max_y_2d + half_size), 0, res - 1);
    if (PRINT_MESSAGE) printf("edge case handled: x0: %u, x1: %u, y0: %u, y1: %u\n", x0, x1, y0, y1);

    // precompute parts of the light color
    pre_light* pre_lights = malloc(sizeof(pre_light) * state->r_spec.n_lights);
    for (int l = 0; l < state->r_spec.n_lights; l++) {
      light_t currentLight = state->r_spec.lights[l];
      pre_lights[l].pos = currentLight.pos;
      pre_lights[l].red = currentLight.intensity.red * currentMat.diffuse.red;
      pre_lights[l].green = currentLight.intensity.green * currentMat.diffuse.green;
      pre_lights[l].blue = currentLight.intensity.blue * currentMat.diffuse.blue;
    }

    // go through each pixel of the current sphere's projection
    for (int y = y0; y <= y1; y++) {
      for (int x = x0; x <= x1; x++) {

      if (colored[y * res + x]) continue;
      
      // compute the ray to the current pixel
      ray_t r = origin_to_pixel(state, x, y);

      // check ray sphere intersection and get the distance
      float t;
      if (!ray_sphere_intersection(&r, &currentSphere, &t)) {
        set_pixel(state, x, y, 0, 0, 0);
        continue;
      }

      // get the normal vector at intersection point and the sphere radius
      vector_t intersection = qadd(r.origin, scale(t, r.dir));
      vector_t normal = qsubtract(intersection, sphereCenter);
      float n_size = qsize(normal);
      normal = scale(1 / n_size, normal);
      
      // color the current pixel
      double red = 0;
      double green = 0;
      double blue = 0;
      for (int j = 0; j < state->r_spec.n_lights; j++) {
        // light_t currentLight = state->r_spec.lights[j];
        pre_light currentLight = pre_lights[j];
        vector_t intersection_to_light = qsubtract(currentLight.pos, intersection);
        if (qdot(normal, intersection_to_light) <= 0)
          continue;

        ray_t lightRay;
        lightRay.origin = intersection;
        lightRay.dir = scale(1 / qsize(intersection_to_light), intersection_to_light);

        // Calculate Lambert diffusion
        float lambert = qdot(lightRay.dir, normal);
        // red += (double)(currentLight.intensity.red * currentMat.diffuse.red * lambert);
        // green += (double)(currentLight.intensity.green * currentMat.diffuse.green * lambert);
        // blue += (double)(currentLight.intensity.blue * currentMat.diffuse.blue * lambert);
        red += (double)(currentLight.red * lambert);
        green += (double)(currentLight.green * lambert);
        blue += (double)(currentLight.blue * lambert);
      }
      set_pixel(state, x, y, red, green, blue);
      colored[y * res + x] = true;
      }
    }
  }
  free(sorted_spheres);
  free(colored);
  return state->img;
}