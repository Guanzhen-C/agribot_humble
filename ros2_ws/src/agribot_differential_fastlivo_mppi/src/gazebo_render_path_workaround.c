#define _GNU_SOURCE

#include <dlfcn.h>
#include <stdlib.h>

typedef int (*get_render_path_type_fn)(const void *);

int agribot_get_render_path_type(const void *render_engine)
    __asm__("_ZNK6gazebo9rendering12RenderEngine17GetRenderPathTypeEv");

int agribot_get_render_path_type(const void *render_engine)
{
  static get_render_path_type_fn original = NULL;

  if (getenv("AGRIBOT_GAZEBO_RENDER_PATH_WORKAROUND") != NULL) {
    return 2;
  }

  if (original == NULL) {
    original = (get_render_path_type_fn)dlsym(
      RTLD_NEXT,
      "_ZNK6gazebo9rendering12RenderEngine17GetRenderPathTypeEv");
  }

  return original != NULL ? original(render_engine) : 0;
}
