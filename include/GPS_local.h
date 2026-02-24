#pragma once

typedef struct {
    double lat0;
    double lon0;
    double cos_lat0;
} gps_origin_t;

void gps_set_origin(gps_origin_t *o, double lat, double lon);

void gps_to_local_xy(
    const gps_origin_t *o,
    double lat,
    double lon,
    float *x,
    float *y);