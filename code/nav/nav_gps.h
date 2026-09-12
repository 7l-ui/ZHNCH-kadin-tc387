#ifndef _NAV_GPS_H_
#define _NAV_GPS_H_

#include "nav_types.h"

void nav_gps_init(void);
void nav_gps_set_min_satellite(uint8 min_satellite);
void nav_gps_set_origin(double latitude_deg, double longitude_deg);
uint8 nav_gps_set_origin_from_current(void);
uint8 nav_gps_has_origin(void);
void nav_gps_local_frame_init(nav_local_frame_t *frame,
                              double origin_latitude_deg,
                              double origin_longitude_deg);
uint8 nav_gps_local_frame_ll_to_xy(const nav_local_frame_t *frame,
                                   double latitude_deg,
                                   double longitude_deg,
                                   nav_point_t *out_point);

uint8 nav_gps_update_sample(const nav_gps_sample_t *input_sample, nav_gps_sample_t *out_sample);
uint8 nav_gps_update_from_gnss(uint32 timestamp_ms, nav_gps_sample_t *out_sample);
void nav_gps_get_last_sample(nav_gps_sample_t *out_sample);
uint8 nav_gps_get_last_local(nav_point_t *out_point);
void nav_gps_ll_to_local(double latitude_deg, double longitude_deg, float *x_m, float *y_m);
float nav_gps_heading_error_deg(float current_yaw_deg, float target_course_deg);

void nav_gps_average_reset(nav_gps_average_t *average, uint16 required_count);
uint8 nav_gps_average_add_sample(nav_gps_average_t *average,
                                 const nav_gps_sample_t *input_sample,
                                 nav_gps_sample_t *out_sample);
uint8 nav_gps_average_add_current(nav_gps_average_t *average,
                                  uint32 timestamp_ms,
                                  nav_gps_sample_t *out_sample);

nav_point_t nav_gps_point_offset(nav_point_t origin, nav_point_t offset);
float nav_gps_point_distance_m(const nav_point_t *a, const nav_point_t *b);
float nav_gps_bearing_deg(const nav_point_t *from, const nav_point_t *to);
uint8 nav_gps_line_from_points(const nav_point_t *start,
                               const nav_point_t *end,
                               nav_line_t *out_line);
float nav_gps_point_to_line_distance_m(const nav_line_t *line,
                                       const nav_point_t *point);
uint8 nav_gps_project_point_to_line(const nav_line_t *line,
                                    const nav_point_t *point,
                                    nav_point_t *out_point);
uint8 nav_gps_arc_from_center_radius(const nav_point_t *center,
                                     float start_deg,
                                     float end_deg,
                                     float radius_m,
                                     nav_arc_dir_t dir,
                                     nav_arc_t *out_arc);
uint8 nav_gps_arc_from_two_points_radius(const nav_point_t *start,
                                         const nav_point_t *end,
                                         float radius_m,
                                         nav_arc_dir_t dir,
                                         nav_arc_t *out_arc);
float nav_gps_point_to_arc_distance_m(const nav_arc_t *arc,
                                      const nav_point_t *point);
float nav_gps_arc_tangent_deg(const nav_arc_t *arc,
                              const nav_point_t *point);

#endif
