#ifndef ACG138_NAVIGATION_MAP_H
#define ACG138_NAVIGATION_MAP_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define NAV_MAP_SIDE 64
#define NAV_MAP_POINTS 4096
#define NAV_RENDER_MAX_CELLS 1000000
#define NAV_MAP_STALE_MS 2000u
struct navigation_map {
    int loaded, available, pose_valid, width, height, has_goal, map_commit_confirmed;
    unsigned path_count, trajectory_count;
    uint64_t updated_monotonic_ms, map_revision;
    int64_t sensor_age_ms;
    double resolution_m, origin[2], pose[3], goal[2], robot_radius_m;
    double path[NAV_MAP_POINTS][2], trajectory[NAV_MAP_POINTS][2];
    char cells[NAV_MAP_SIDE*NAV_MAP_SIDE+1];
    /* Display-only original ROS grid; never used for goal admission or control. */
    unsigned render_width,render_height;
    uint64_t render_revision;
    char render_session[129];
    double render_resolution,render_origin[3];
    char render_cells[NAV_RENDER_MAX_CELLS+1];
    int display_gray_ready;
    uint8_t display_gray[NAV_RENDER_MAX_CELLS];
    double display_x,display_y,display_span;
    char state[80], reason[160], session_id[129];
};
struct navigation_reader;
/* The worker alone performs file I/O and JSON parsing, at most ten times/second.
   UI snapshot copies use trylock and keep the previous frame on contention. */
struct navigation_reader *navigation_reader_start(const char *path);
void navigation_reader_stop(struct navigation_reader *reader);
int navigation_reader_snapshot(struct navigation_reader *reader, struct navigation_map *out);
int navigation_map_parse(const char *json,size_t length,struct navigation_map *out);
int navigation_map_read(const char *path,struct navigation_map *out);
static inline int navigation_map_fresh(const struct navigation_map *m,uint64_t now) {
    return m&&m->loaded&&m->available&&m->pose_valid&&now>=m->updated_monotonic_ms&&
        now-m->updated_monotonic_ms<=NAV_MAP_STALE_MS&&m->sensor_age_ms>=0&&m->sensor_age_ms<=1000;
}
#ifdef __cplusplus
}
#endif
#endif
