const float* render(renderer_state_t *state, const sphere_t *spheres, int n_spheres) {

  printf("This is the old render function in the archieve!\n");
  
  sphere_t* sorted_spheres = sort(state, spheres, n_spheres);
  assert(sorted_spheres != NULL);
  ray_t r;

  for (int y = 0; y < state->r_spec.resolution; y++) {
    for (int x = 0; x < state->r_spec.resolution; x++) {
      r = origin_to_pixel(state, x, y);

      // Finds the first ray-sphere intersection.
      // Since spheres are sorted, the first intersection will also
      // be the closest intersection.
      float t = INFINITY;
      int currentSphere = -1;

      for (int i = 0; i < n_spheres; i++) {
        if (ray_sphere_intersection(&r, &sorted_spheres[i], &t)) {
          currentSphere = i;
          break;
        }
      }

      // If ray does not intersect any sphere, color the pixel black
      if (currentSphere == -1) {
        set_pixel(state, x, y, 0, 0, 0);
        continue;
      }

      material_t currentMat = sorted_spheres[currentSphere].mat;

      vector_t intersection = qadd(r.origin, scale(t, r.dir));

      // Normal vector at intersection point, perpendicular to the surface
      // of the sphere
      vector_t normal = qsubtract(intersection, sorted_spheres[currentSphere].pos);
      float n_size = qsize(normal);
      // Note: n_size should be the radius of the sphere, which is nonzero.
      normal = scale(1 / n_size, normal);

      double red = 0;
      double green = 0;
      double blue = 0;

      for (int j = 0; j < state->r_spec.n_lights; j++) {
        light_t currentLight = state->r_spec.lights[j];
        vector_t intersection_to_light = qsubtract(currentLight.pos, intersection);
        if (qdot(normal, intersection_to_light) <= 0)
          continue;

        ray_t lightRay;
        lightRay.origin = intersection;
        lightRay.dir = scale(1 / qsize(intersection_to_light), intersection_to_light);

        // Calculate Lambert diffusion
        float lambert = qdot(lightRay.dir, normal);
        red += (double)(currentLight.intensity.red * currentMat.diffuse.red *
                        lambert);
        green += (double)(currentLight.intensity.green *
                          currentMat.diffuse.green * lambert);
        blue += (double)(currentLight.intensity.blue * currentMat.diffuse.blue *
                         lambert);
      }

      set_pixel(state, x, y, red, green, blue);
    }
  }
  free(sorted_spheres);
  return state->img;
}