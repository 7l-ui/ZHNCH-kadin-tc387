#include "nav_gps.h"
#include "zf_device_gnss.h"
#include <math.h>

#define NAV_GPS_DEG_TO_M       (111319.49079327358)
#define NAV_GPS_DEFAULT_MIN_SAT (4U)
#define NAV_GPS_LINE_MIN_LEN_M  (0.05f)

static uint8 g_nav_gps_min_satellite = NAV_GPS_DEFAULT_MIN_SAT;
static nav_local_frame_t g_nav_gps_frame = {0};
static nav_gps_sample_t g_nav_gps_last = {0};

static float NavGps_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static uint8 NavGps_CheckSample(const nav_gps_sample_t *input_sample,
                                nav_gps_sample_t *out_sample)
{
    nav_gps_sample_t sample;

    if (input_sample == NULL)
    {
        return 0U;
    }

    sample = *input_sample;
    sample.course_deg = NavGps_Wrap180(sample.course_deg);
    sample.valid = ((0U != sample.valid) &&
                    (sample.satellite_used >= g_nav_gps_min_satellite) &&
                    (0.0 != sample.latitude_deg) &&
                    (0.0 != sample.longitude_deg)) ? 1U : 0U;

    if (out_sample != NULL)
    {
        *out_sample = sample;
    }

    return sample.valid;
}

void nav_gps_init(void)
{
    g_nav_gps_frame.valid = 0U;
    g_nav_gps_frame.origin_latitude_deg = 0.0;
    g_nav_gps_frame.origin_longitude_deg = 0.0;
    g_nav_gps_frame.meters_per_degree_lat = NAV_GPS_DEG_TO_M;
    g_nav_gps_frame.meters_per_degree_lon = NAV_GPS_DEG_TO_M;
    g_nav_gps_min_satellite = NAV_GPS_DEFAULT_MIN_SAT;
    g_nav_gps_last.valid = 0U;
}

void nav_gps_set_min_satellite(uint8 min_satellite)
{
    g_nav_gps_min_satellite = min_satellite;
}

void nav_gps_set_origin(double latitude_deg, double longitude_deg)
{
    nav_gps_local_frame_init(&g_nav_gps_frame, latitude_deg, longitude_deg);
}

uint8 nav_gps_set_origin_from_current(void)
{
    if ((0U == gnss.state) || (0.0 == gnss.latitude) || (0.0 == gnss.longitude))
    {
        return 0U;
    }

    nav_gps_set_origin(gnss.latitude, gnss.longitude);
    return 1U;
}

uint8 nav_gps_has_origin(void)
{
    return g_nav_gps_frame.valid;
}

void nav_gps_local_frame_init(nav_local_frame_t *frame,
                              double origin_latitude_deg,
                              double origin_longitude_deg)
{
    if (frame == NULL)
    {
        return;
    }

    frame->valid = 1U;
    frame->origin_latitude_deg = origin_latitude_deg;
    frame->origin_longitude_deg = origin_longitude_deg;
    frame->meters_per_degree_lat = NAV_GPS_DEG_TO_M;
    frame->meters_per_degree_lon = NAV_GPS_DEG_TO_M *
                                    cos(origin_latitude_deg * (double)NAV_PI / 180.0);
}

uint8 nav_gps_local_frame_ll_to_xy(const nav_local_frame_t *frame,
                                   double latitude_deg,
                                   double longitude_deg,
                                   nav_point_t *out_point)
{
    if ((frame == NULL) || (out_point == NULL) || (0U == frame->valid))
    {
        return 0U;
    }

    out_point->x_m = (float)((longitude_deg - frame->origin_longitude_deg) *
                             frame->meters_per_degree_lon);
    out_point->y_m = (float)((latitude_deg - frame->origin_latitude_deg) *
                             frame->meters_per_degree_lat);
    return 1U;
}

void nav_gps_ll_to_local(double latitude_deg, double longitude_deg, float *x_m, float *y_m)
{
    nav_point_t point = {0};

    if (0U == nav_gps_local_frame_ll_to_xy(&g_nav_gps_frame,
                                           latitude_deg,
                                           longitude_deg,
                                           &point))
    {
        if (x_m != NULL) *x_m = 0.0f;
        if (y_m != NULL) *y_m = 0.0f;
        return;
    }

    if (x_m != NULL) *x_m = point.x_m;
    if (y_m != NULL) *y_m = point.y_m;
}

uint8 nav_gps_update_sample(const nav_gps_sample_t *input_sample, nav_gps_sample_t *out_sample)
{
    nav_gps_sample_t sample;

    if (0U == NavGps_CheckSample(input_sample, &sample))
    {
        if (input_sample != NULL)
        {
            sample = *input_sample;
            sample.course_deg = NavGps_Wrap180(sample.course_deg);
            sample.valid = 0U;
            g_nav_gps_last = sample;
            if (out_sample != NULL)
            {
                *out_sample = sample;
            }
        }
        return 0U;
    }

    if (0U == g_nav_gps_frame.valid)
    {
        nav_gps_set_origin(sample.latitude_deg, sample.longitude_deg);
    }

    nav_gps_ll_to_local(sample.latitude_deg, sample.longitude_deg, &sample.x_m, &sample.y_m);

    g_nav_gps_last = sample;

    if (out_sample != NULL)
    {
        *out_sample = sample;
    }

    return sample.valid;
}

uint8 nav_gps_update_from_gnss(uint32 timestamp_ms, nav_gps_sample_t *out_sample)
{
    nav_gps_sample_t sample = {0};

    sample.valid = (0U != gnss.state) ? 1U : 0U;
    sample.timestamp_ms = timestamp_ms;
    sample.latitude_deg = gnss.latitude;
    sample.longitude_deg = gnss.longitude;
    sample.satellite_used = gnss.satellite_used;
    sample.speed_mps = gnss.speed / 3.6f;
    sample.course_deg = gnss.direction;

    return nav_gps_update_sample(&sample, out_sample);
}

void nav_gps_get_last_sample(nav_gps_sample_t *out_sample)
{
    if (out_sample != NULL)
    {
        *out_sample = g_nav_gps_last;
    }
}

uint8 nav_gps_get_last_local(nav_point_t *out_point)
{
    if ((out_point == NULL) || (0U == g_nav_gps_last.valid))
    {
        return 0U;
    }

    out_point->x_m = g_nav_gps_last.x_m;
    out_point->y_m = g_nav_gps_last.y_m;
    return 1U;
}

float nav_gps_heading_error_deg(float current_yaw_deg, float target_course_deg)
{
    float diff = target_course_deg - current_yaw_deg;
    float rad = diff * NAV_PI / 180.0f;
    return atan2f(sinf(rad), cosf(rad)) * 180.0f / NAV_PI;
}

void nav_gps_average_reset(nav_gps_average_t *average, uint16 required_count)
{
    if (average == NULL)
    {
        return;
    }

    average->valid = 0U;
    average->sample_count = 0U;
    average->required_count = (required_count == 0U) ? 1U : required_count;
    average->first_timestamp_ms = 0U;
    average->last_timestamp_ms = 0U;
    average->latitude_sum_deg = 0.0;
    average->longitude_sum_deg = 0.0;
    average->speed_sum_mps = 0.0f;
    average->course_sin_sum = 0.0f;
    average->course_cos_sum = 0.0f;
    average->satellite_sum = 0U;
}

uint8 nav_gps_average_add_sample(nav_gps_average_t *average,
                                 const nav_gps_sample_t *input_sample,
                                 nav_gps_sample_t *out_sample)
{
    nav_gps_sample_t checked_sample;
    nav_gps_sample_t averaged_sample;
    float course_rad;

    if ((average == NULL) || (input_sample == NULL))
    {
        return 0U;
    }

    if (average->required_count == 0U)
    {
        nav_gps_average_reset(average, 1U);
    }

    if (0U == NavGps_CheckSample(input_sample, &checked_sample))
    {
        return 0U;
    }

    if (average->sample_count == 0U)
    {
        average->first_timestamp_ms = checked_sample.timestamp_ms;
    }

    course_rad = checked_sample.course_deg * NAV_PI / 180.0f;
    average->sample_count++;
    average->last_timestamp_ms = checked_sample.timestamp_ms;
    average->latitude_sum_deg += checked_sample.latitude_deg;
    average->longitude_sum_deg += checked_sample.longitude_deg;
    average->speed_sum_mps += checked_sample.speed_mps;
    average->course_sin_sum += sinf(course_rad);
    average->course_cos_sum += cosf(course_rad);
    average->satellite_sum += checked_sample.satellite_used;

    if (average->sample_count < average->required_count)
    {
        return 0U;
    }

    averaged_sample.valid = 1U;
    averaged_sample.timestamp_ms = average->last_timestamp_ms;
    averaged_sample.latitude_deg = average->latitude_sum_deg / (double)average->sample_count;
    averaged_sample.longitude_deg = average->longitude_sum_deg / (double)average->sample_count;
    averaged_sample.speed_mps = average->speed_sum_mps / (float)average->sample_count;
    averaged_sample.course_deg = atan2f(average->course_sin_sum,
                                        average->course_cos_sum) *
                                180.0f / NAV_PI;
    averaged_sample.satellite_used = (uint8)(average->satellite_sum /
                                             (uint32)average->sample_count);
    averaged_sample.x_m = 0.0f;
    averaged_sample.y_m = 0.0f;
    if (0U != g_nav_gps_frame.valid)
    {
        nav_gps_ll_to_local(averaged_sample.latitude_deg,
                            averaged_sample.longitude_deg,
                            &averaged_sample.x_m,
                            &averaged_sample.y_m);
    }
    average->valid = averaged_sample.valid;

    if (out_sample != NULL)
    {
        *out_sample = averaged_sample;
    }

    return averaged_sample.valid;
}

uint8 nav_gps_average_add_current(nav_gps_average_t *average,
                                  uint32 timestamp_ms,
                                  nav_gps_sample_t *out_sample)
{
    nav_gps_sample_t sample = {0};

    sample.valid = (0U != gnss.state) ? 1U : 0U;
    sample.timestamp_ms = timestamp_ms;
    sample.latitude_deg = gnss.latitude;
    sample.longitude_deg = gnss.longitude;
    sample.satellite_used = gnss.satellite_used;
    sample.speed_mps = gnss.speed / 3.6f;
    sample.course_deg = gnss.direction;

    return nav_gps_average_add_sample(average, &sample, out_sample);
}

nav_point_t nav_gps_point_offset(nav_point_t origin, nav_point_t offset)
{
    nav_point_t point;

    point.x_m = origin.x_m + offset.x_m;
    point.y_m = origin.y_m + offset.y_m;
    return point;
}

float nav_gps_point_distance_m(const nav_point_t *a, const nav_point_t *b)
{
    float dx;
    float dy;

    if ((a == NULL) || (b == NULL))
    {
        return 0.0f;
    }

    dx = b->x_m - a->x_m;
    dy = b->y_m - a->y_m;
    return sqrtf(dx * dx + dy * dy);
}

float nav_gps_bearing_deg(const nav_point_t *from, const nav_point_t *to)
{
    if ((from == NULL) || (to == NULL))
    {
        return 0.0f;
    }

    return NavGps_Wrap180(-atan2f(to->x_m - from->x_m,
                                  to->y_m - from->y_m) *
                          180.0f / NAV_PI);
}

uint8 nav_gps_line_from_points(const nav_point_t *start,
                               const nav_point_t *end,
                               nav_line_t *out_line)
{
    float dx;
    float dy;
    float len_m;

    if ((start == NULL) || (end == NULL) || (out_line == NULL))
    {
        return 0U;
    }

    dx = end->x_m - start->x_m;
    dy = end->y_m - start->y_m;
    len_m = sqrtf(dx * dx + dy * dy);

    out_line->start = *start;
    out_line->end = *end;
    out_line->a = dy;
    out_line->b = -dx;
    out_line->c = dx * start->y_m - dy * start->x_m;
    out_line->valid = (len_m >= NAV_GPS_LINE_MIN_LEN_M) ? 1U : 0U;

    return out_line->valid;
}

float nav_gps_point_to_line_distance_m(const nav_line_t *line,
                                       const nav_point_t *point)
{
    float denom;
    float signed_distance;

    if ((line == NULL) || (point == NULL) || (0U == line->valid))
    {
        return 0.0f;
    }

    denom = sqrtf(line->a * line->a + line->b * line->b);
    if (denom < 1.0e-6f)
    {
        return 0.0f;
    }

    signed_distance = (line->a * point->x_m + line->b * point->y_m + line->c) / denom;
    return -signed_distance;
}

uint8 nav_gps_project_point_to_line(const nav_line_t *line,
                                    const nav_point_t *point,
                                    nav_point_t *out_point)
{
    float denom;
    float distance_scale;

    if ((line == NULL) || (point == NULL) || (out_point == NULL) ||
        (0U == line->valid))
    {
        return 0U;
    }

    denom = line->a * line->a + line->b * line->b;
    if (denom < 1.0e-6f)
    {
        return 0U;
    }

    distance_scale = (line->a * point->x_m + line->b * point->y_m + line->c) / denom;
    out_point->x_m = point->x_m - line->a * distance_scale;
    out_point->y_m = point->y_m - line->b * distance_scale;
    return 1U;
}

uint8 nav_gps_arc_from_center_radius(const nav_point_t *center,
                                     float start_deg,
                                     float end_deg,
                                     float radius_m,
                                     nav_arc_dir_t dir,
                                     nav_arc_t *out_arc)
{
    if ((center == NULL) || (out_arc == NULL) || (radius_m <= 0.0f))
    {
        return 0U;
    }

    out_arc->center = *center;
    out_arc->start_deg = NavGps_Wrap180(start_deg);
    out_arc->end_deg = NavGps_Wrap180(end_deg);
    out_arc->radius_m = radius_m;
    out_arc->dir = dir;
    out_arc->valid = 1U;
    return 1U;
}

uint8 nav_gps_arc_from_two_points_radius(const nav_point_t *start,
                                         const nav_point_t *end,
                                         float radius_m,
                                         nav_arc_dir_t dir,
                                         nav_arc_t *out_arc)
{
    nav_point_t midpoint;
    nav_point_t center;
    float dx;
    float dy;
    float chord_m;
    float half_chord_m;
    float center_offset_m;
    float unit_x;
    float unit_y;
    float normal_x;
    float normal_y;

    if ((start == NULL) || (end == NULL) || (out_arc == NULL) ||
        (radius_m <= 0.0f))
    {
        return 0U;
    }

    dx = end->x_m - start->x_m;
    dy = end->y_m - start->y_m;
    chord_m = sqrtf(dx * dx + dy * dy);
    if ((chord_m < NAV_GPS_LINE_MIN_LEN_M) || (chord_m > (2.0f * radius_m)))
    {
        out_arc->valid = 0U;
        return 0U;
    }

    half_chord_m = chord_m * 0.5f;
    center_offset_m = sqrtf(radius_m * radius_m - half_chord_m * half_chord_m);
    unit_x = dx / chord_m;
    unit_y = dy / chord_m;
    normal_x = -unit_y;
    normal_y = unit_x;

    midpoint.x_m = (start->x_m + end->x_m) * 0.5f;
    midpoint.y_m = (start->y_m + end->y_m) * 0.5f;

    if (dir == NAV_ARC_DIR_CW)
    {
        center.x_m = midpoint.x_m + normal_x * center_offset_m;
        center.y_m = midpoint.y_m + normal_y * center_offset_m;
    }
    else
    {
        center.x_m = midpoint.x_m - normal_x * center_offset_m;
        center.y_m = midpoint.y_m - normal_y * center_offset_m;
    }

    return nav_gps_arc_from_center_radius(&center,
                                          nav_gps_bearing_deg(&center, start),
                                          nav_gps_bearing_deg(&center, end),
                                          radius_m,
                                          dir,
                                          out_arc);
}

float nav_gps_point_to_arc_distance_m(const nav_arc_t *arc,
                                      const nav_point_t *point)
{
    float distance_to_center;
    float radial_error;

    if ((arc == NULL) || (point == NULL) || (0U == arc->valid))
    {
        return 0.0f;
    }

    distance_to_center = nav_gps_point_distance_m(&arc->center, point);
    radial_error = arc->radius_m - distance_to_center;

    return (arc->dir == NAV_ARC_DIR_CW) ? radial_error : -radial_error;
}

float nav_gps_arc_tangent_deg(const nav_arc_t *arc,
                              const nav_point_t *point)
{
    float radial_deg;

    if ((arc == NULL) || (point == NULL) || (0U == arc->valid))
    {
        return 0.0f;
    }

    radial_deg = nav_gps_bearing_deg(&arc->center, point);
    if (arc->dir == NAV_ARC_DIR_CW)
    {
        return NavGps_Wrap180(radial_deg - 90.0f);
    }

    return NavGps_Wrap180(radial_deg + 90.0f);
}
