#ifndef __POINT_TYPES__
#define __POINT_TYPES__

#include <stdint.h>

namespace obvious {

struct PointXyz {
    float x;
    float y;
    float z;
};

struct PointXyzRgb {
    float x;
    float y;
    float z;
    char r;
    char g;
    char b;
};

struct PointXyzI {
    float x;
    float y;
    float z;
    uint16_t i;
};

struct PointXyzT {
    float x;
    float y;
    float z;
    uint16_t t;
};

struct PointXyzRgbT {
    float x;
    float y;
    float z;
    char r;
    char g;
    char b;
    uint16_t t;
};

struct PointXyzRgbI {
    float x;
    float y;
    float z;
    char r;
    char g;
    char b;
    uint16_t i;
};

struct PointXyzRgbIT {
    float x;
    float y;
    float z;
    char r;
    char g;
    char b;
    uint16_t i;
    uint16_t t;
};

}

#endif
