#include <math.h>
#include "gps_local.h"

#define EARTH_RADIUS 6378137.0
#define DEG2RAD (M_PI / 180.0)

void gps_set_origin(gps_origin_t *o, double lat, double lon)
{
    o->lat0 = lat;
    o->lon0 = lon;
    o->cos_lat0 = cos(lat * DEG2RAD);
}

void gps_to_local_xy(
    const gps_origin_t *o,
    double lat,
    double lon,
    float *x,
    float *y)
{
    double dlat = (lat - o->lat0) * DEG2RAD;
    double dlon = (lon - o->lon0) * DEG2RAD;

    *y = (float)(dlat * EARTH_RADIUS);
    *x = (float)(dlon * EARTH_RADIUS * o->cos_lat0);
}