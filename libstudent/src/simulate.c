/**
 * Author: Isabel Rosa, isrosa@mit.edu
 **/

#include <assert.h>
#include <cilk/cilk.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "../../common/simulate.h"
#include "../include/misc_utils.h"

typedef struct simulator_state {
  simulator_spec_t s_spec;
  sphere_t *spheres;
} simulator_state_t;

typedef struct {
  float min_x, max_x, min_y, max_y, min_z, max_z;
  int sphere_index;
} bounding_box;

int OLD_CHECK = 0;
int BOX_CHECK = 0;

simulator_state_t* init_simulator(const simulator_spec_t *spec) {
  simulator_state_t *state = (simulator_state_t*)malloc(sizeof(simulator_state_t));
  state->s_spec = *spec;
  state->spheres = malloc(2 * spec->n_spheres * sizeof(sphere_t));
  assert(state->spheres != NULL);
  memcpy(state->spheres, spec->spheres, sizeof(sphere_t) * spec->n_spheres);
  memcpy(state->spheres + state->s_spec.n_spheres, spec->spheres, sizeof(sphere_t) * state->s_spec.n_spheres);
  return state;
}

void destroy_simulator(simulator_state_t* state) {
  free(state->spheres);
  free(state);
}

void update_accel_sphere(sphere_t *spheres, int n_spheres, double g, int i) {
  double rx = 0;
  double ry = 0;
  double rz = 0;

  const vector_t zero_vec = {0, 0, 0};
  spheres[i + n_spheres].accel = zero_vec;
  for (int j = 0; j < n_spheres; j++) {
    if (i != j) {
      vector_t i_minus_j = qsubtract(spheres[i].pos, spheres[j].pos);
      vector_t j_minus_i = scale(-1, i_minus_j);
      vector_t force =
          scale(g * spheres[j].mass / pow(qsize(i_minus_j), 3), j_minus_i);
      rx += (double)force.x;
      ry += (double)force.y;
      rz += (double)force.z;
    }
  }
  const vector_t v = {.x = rx, .y = ry, .z = rz};
  spheres[i + n_spheres].accel = v;
}

void update_accelerations(sphere_t *spheres, int n_spheres, double g) {
  for (int i = 0; i < n_spheres; i++) {
    update_accel_sphere(spheres, n_spheres, g, i);
  }
}

void update_velocities(sphere_t *spheres, int n_spheres, float t) {
  for (int i = 0; i < n_spheres; i++) {
    spheres[i + n_spheres].vel = qadd(spheres[i].vel, scale(t, spheres[i].accel));
  }
}

void update_positions(sphere_t *spheres, int n_spheres, float t) {
  for (int i = 0; i < n_spheres; i++) {
    spheres[i + n_spheres].pos = qadd(spheres[i].pos, scale(t, spheres[i].vel));
  }
}

// runs simulation for minCollisionTime timesteps
// perform collision between spheres at indices i and j
void do_ministep(sphere_t *spheres, int n_spheres, double g, float minCollisionTime, int i, int j) {
  update_accelerations(spheres, n_spheres, g);
  update_velocities(spheres, n_spheres, minCollisionTime);
  update_positions(spheres, n_spheres, minCollisionTime);

  for (int k = 0; k < n_spheres; k++) {
    spheres[k] = spheres[k + n_spheres];
  }

  if (i == -1 || j == -1) {
    return;
  }

  vector_t distVec = qsubtract(spheres[i].pos, spheres[j].pos);
  float scale1 = 2 * spheres[j].mass /
                 (float)((double)spheres[i].mass + (double)spheres[j].mass);
  float scale2 = 2 * spheres[i].mass /
                 (float)((double)spheres[i].mass + (double)spheres[j].mass);
  float distNorm = qdot(distVec, distVec);
  vector_t velDiff = qsubtract(spheres[i].vel, spheres[j].vel);
  vector_t scaledDist = scale(qdot(velDiff, distVec) / distNorm, distVec);
  spheres[i].vel = qsubtract(spheres[i].vel, scale(scale1, scaledDist));
  spheres[j].vel = qsubtract(spheres[j].vel, scale(-1 * scale2, scaledDist));
}

// Check if the spheres at indices i and j collide in the next
// timeToCollision timesteps
// 
// If so, modifies timeToCollision to be the time until spheres i and j collide.
int check_for_collision(sphere_t *spheres, int i, int j, float *timeToCollision) {
  OLD_CHECK++;
  vector_t distVec = qsubtract(spheres[i].pos, spheres[j].pos);
  float dist = qsize(distVec);
  float sumRadii = (float)((double)spheres[i].r + (double)spheres[j].r);

  // Shift frame of reference to act like sphere i is stationary
  // Not adjusting for acceleration because our simulation does not adjust for acceleration
  vector_t movevec = qsubtract(spheres[j].vel, spheres[i].vel);

  // Distance that sphere j moves in timeToCollision time
  float moveDist = (float)((double)qsize(movevec) * (double)*timeToCollision);

  // Break if the length the sphere moves in timeToCollision time is less than
  // distance between the centers of these spheres minus their radii
  if ((double)moveDist < (double)dist - (double)sumRadii ||
      (movevec.x == 0 && movevec.y == 0 && movevec.z == 0)) {
    return 0;
  }

  vector_t unitMovevec = scale(1 / qsize(movevec), movevec);

  // distAlongMovevec = ||distVec|| * cos(angle between unitMovevec and distVec)
  float distAlongMovevec = qdot(unitMovevec, distVec);

  // Check that sphere j is moving towards sphere i
  if (distAlongMovevec <= 0) {
    return 0;
  }

  float jToMovevecDistSq =
      (float)((double)(dist * dist) -
              (double)(distAlongMovevec * distAlongMovevec));

  // Break if the closest that sphere j will get to sphere i is more than
  // the sum of their radii
  float sumRadiiSquared = sumRadii * sumRadii;
  if (jToMovevecDistSq >= sumRadiiSquared) {
    return 0;
  }

  // We now have jToMovevecDistSq and sumRadii, two sides of a right triangle.
  // Use these to find the third side, sqrt(T)
  float extraDist = (float)((double)sumRadiiSquared - (double)jToMovevecDistSq);

  if (extraDist < 0) {
    return 0;
  }

  // Draw out the spheres to check why this is the distance sphere j moves
  // before hitting sphere i;)
  float distance = (float)((double)distAlongMovevec - (double)sqrt(extraDist));

  // Break if the distance sphere j has to move to touch sphere i is too big
  if (distance < 0 || moveDist < distance) {
    return 0;
  }

  *timeToCollision = distance / qsize(movevec);
  return 1;
}

bounding_box generate_box(sphere_t* currentSphere, float t, int i) {
  vector_t start_pos = currentSphere->pos;
  vector_t end_pos = qadd(currentSphere->pos, scale(t, currentSphere->vel));
  float r = currentSphere->r;

  bounding_box b;
  b.min_x = fmin(start_pos.x, end_pos.x) - r;
  b.max_x = fmax(start_pos.x, end_pos.x) + r;
  b.min_y = fmin(start_pos.y, end_pos.y) - r;
  b.max_y = fmax(start_pos.y, end_pos.y) + r;
  b.min_z = fmin(start_pos.z, end_pos.z) - r;
  b.max_z = fmax(start_pos.z, end_pos.z) + r;
  b.sphere_index = i;
  return b;
}

bool check_box_collision(bounding_box* a, bounding_box* b) {
  BOX_CHECK++;
  return !(a->max_x < b->min_x || a->min_x > b->max_x ||
           a->max_y < b->min_y || a->min_y > b->max_y ||
           a->max_z < b->min_z || a->min_z > b->max_z);
}

int compare_boxes_by_x(const void* a, const void* b) {
  bounding_box* box_a = (bounding_box*) a;
  bounding_box* box_b = (bounding_box*) b;

  if (box_a->min_x < box_b->min_x) return -1;
  if (box_b->min_x < box_a->min_x) return 1;
  return 0;
}

void do_timestep(simulator_state_t* state, float timeStep, bounding_box* boxes) {
  float timeLeft = timeStep;
  int n_spheres = state->s_spec.n_spheres;

  // If collisions are getting too frequent, we cut time step early
  // This allows for smoother rendering without losing accuracy
  while (timeLeft > 0.000001) {
    float minCollisionTime = timeLeft;
    int indexCollider1 = -1;
    int indexCollider2 = -1;
    
    for (int i = 0; i < n_spheres; i++) {
      boxes[i] = generate_box(&state->spheres[i], minCollisionTime, i);
    }
    qsort(boxes, n_spheres, sizeof(bounding_box), compare_boxes_by_x);

    for (int i = 0; i < n_spheres; i++) {
      bounding_box* box1 = &boxes[i];
      for (int j = i + 1; j < n_spheres; j++) {
        bounding_box* box2 = &boxes[j];
        if (box2->min_x > box1->max_x) break;
        if (box2->min_y > box1->max_y || box2->max_y < box1->min_y) continue;
        if (box2->min_z > box1->max_z || box2->max_z < box1->min_z) continue;

        if (check_box_collision(box1, box2)) {
          if (check_for_collision(state->spheres, box1->sphere_index, box2->sphere_index, &minCollisionTime)) {
            indexCollider1 = box1->sphere_index;
            indexCollider2 = box2->sphere_index;
          }
        }
      }
    }

    do_ministep(state->spheres, state->s_spec.n_spheres, state->s_spec.g, minCollisionTime, indexCollider1, indexCollider2);

    timeLeft = timeLeft - minCollisionTime;
  }
}

sphere_t* simulate(simulator_state_t* state) {
  OLD_CHECK = 0;
  BOX_CHECK = 0;
  int n_spheres = state->s_spec.n_spheres;
  float timeStep = n_spheres > 1 ? (1 / log(n_spheres)) : 1;
  bounding_box* boxes = malloc(sizeof(bounding_box) * n_spheres);
  do_timestep(state, timeStep, boxes);
  // printf("OLD_CHECK: %d\n", OLD_CHECK);
  // printf("BOX_CHECK: %d\n", BOX_CHECK);
  return state->spheres;
}
