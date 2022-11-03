#ifndef ISL_INCLUDE_COLLISION2D_H_
#define ISL_INCLUDE_COLLISION2D_H_

/* isl_collision2d - v0.1 public domain 2d collision detection library  
                        no warranty implied; use at your own risk
   
   Do this:
       #define ISL_COLLISION2D_IMPLEMENTATION
   before you include this file in *one* C or C++ file to create the implementation.

   To static link also add:
       #define ISL_COLLISION2D_STATIC

   QUICK NOTES:

   USAGE:

   author: Ilya Kolbin (iskolbin@gmail.com)
   url: https://github.com/iskolbin/isl_collision2d
   git: git@github.com:iskolbin/isl_collision2d

   LICENSE:
     See end of file for license information.
*/
#ifndef ISL_COLLISION2D_DOUBLE
#define islc2d_float float
#define islc2d__sqrt sqrtf
#define islc2d__sin  sinf
#define islc2d__cos  cosf
#else
#define islc2d_float double
#define islc2d__sqrt sqrt
#define islc2d__sin  sin
#define islc2d__cos  cos
#endif

#ifndef ISLC2D_DEF
#ifdef ISL_COLLISION2D_STATIC
#define ISLC2D_DEF static
#else
#define ISLC2D_DEF extern
#endif
#endif

#ifndef ISL_COLLISION2D_VEC2
struct islc2d_vec2 {
	islc2d_float x;
	islc2d_float y;
};
#else
#define islc2d_vec2 ISL_COLLISION2D_VEC2
#endif

#ifndef ISL_COLLISION2D_VEC3
struct islc2d_vec3 {
	islc2d_float x;
	islc2d_float y;
	islc2d_float z;
};
#else
#define islc2d_vec3 ISL_COLLISION2D_VEC3
#endif

#ifndef ISL_COLLISION2D_RECT
struct islc2d_rect {
	islc2d_float x;
	islc2d_float y;
	islc2d_float width;
	islc2d_float height;
};
#else
#define islc2d_rect ISL_COLLISION2D_RECT
#endif

#ifdef __cplusplus
extern "C" {
#endif

ISLC2D_DEF bool islc2d_check_polygon_rect(struct islc2d_vec2 *points, int point_count, struct islc2d_rect rect, struct islc2d_vec3 *result);
ISLC2D_DEF bool islc2d_check_polygon_circle(struct islc2d_vec *points, int point_count, struct islc2d_vec2 c, islc2d_float r, struct islc2d_vec3 *result);
ISLC2D_DEF bool islc2d_check_polygon_rect_prepared(struct islc2d_vec2 *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect bbox, int point_count, struct islc2d_rect rect, struct islc2d_vec3 *result);
ISLC2D_DEF bool islc2d_check_polygon_circle_prepared(struct islc2d_vec *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect bbox, int point_count, struct islc2d_vec2 c, islc2d_float r, struct islc2d_vec3 *result);
ISLC2D_DEF bool islc2d_check_rect_circle(struct islc2d_rect rect, struct islc2d_vec2 c, islc2d_float r, struct islc2d_vec3 *result);
ISLC2D_DEF bool islc2d_check_rects(struct islc2d_rect rect1, struct islc2d_rect rect2, struct islc2d_vec3 *result);
ISLC2D_DEF bool islc2d_check_circles(struct islc2d_vec2 c1, islc2d_float r1, struct islc2d_vec2 c2, islc2d_float r2, struct islc2d_vec3 *result);
ISLC2D_DEF bool islc2d_check_polygons(struct islc2d_vec2 *points1, int point_count1, struct islc2d_vec2 *points2, int point_count2, struct islc2d_vec3 *result);
ISLC2D_DEF void islc2d_prepare(struct islc2d_vec2 *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect *bbox, int point_count);
ISLC2D_DEF void islc2d_prepare_fixed_normals(struct islc2d_vec2 *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect *bbox, int point_count);
ISLC2D_DEF bool islc2d_check_polygons_prepared(struct islc2d_vec2 *points1, struct islc2d_vec2 *ns1, struct islc2d_vec2 *minmax1, struct islc2d_rect bbox1, int point_count1, struct islc2d_vec2 *points2, struct islc2d_vec2 *ns2, struct islc2d_vec2 *minmax2, struct islc2d_rect bbox2, int point_count2, struct islc2d_vec3 *result);
ISLC2D_DEF void islc2d_move_polygon(struct islc2d_vec2 *points, int point_count, struct islc2d_vec2 dxy);
ISLC2D_DEF void islc2d_rotate_polygon(struct islc2d_vec2 *points, int point_count, islc2d_float angle, struct islc2d_vec2 origin);
ISLC2D_DEF void islc2d_scale_polygon(struct islc2d_vec2 *points, int point_count, struct islc2d_vec2 scale, struct islc2d_vec2 origin);
ISLC2D_DEF struct islc2d_vec2 islc2d_polygon_center(struct islc2d_vec2 *points, int point_count);

#ifdef __cplusplus
}
#endif
#endif // ISL_INCLUDE_COLLISION2D_H_

#ifdef ISL_COLLISION2D_IMPLEMENTATION
#ifndef ISL_COLLISION2D_IMPLEMENTATION_ONCE
#define ISL_COLLISION2D_IMPLEMENTATION_ONCE
#else
#error "ISL_COLLISION2D_IMPLEMENTATION should be defined once"
#endif

#include <float.h>
#include <math.h>

static islc2d_float islc2d__dot_product(struct islc2d_vec2 v1, struct islc2d_vec2 v2);
static struct islc2d_vec2 islc2d__normalize(struct islc2d_vec2 v);

islc2d_float islc2d__dot_product(struct islc2d_vec2 v1, struct islc2d_vec2 v2) {
	return v1.x * v2.x + v1.y * v2.y;
}

struct islc2d_vec2 islc2d__normalize(struct islc2d_vec2 v) {
	Vector2 result = {0};
	float length = islc2d__sqrt((v.x * v.x) + (v.y * v.y));
	if (length > 0.0) {
		float ilength = 1.0f / length;
		result.x = v.x * ilength;
		result.y = v.y * ilength;
	}
	return result;
}

struct islc2d_vec2 islc2d_polygon_center(struct islc2d_vec2 *points, int point_count) {
	islc2d_float cx = 0.0f, cy = 0.0f, k = 1.0f / point_count;
	for (int i = 0; i < point_count; i++) {
		cx += points[i].x;
		cy += points[i].y;
	}
	return (struct islc2d_vec2) {cx * k, cy * k};
}

void islc2d_move_polygon(struct islc2d_vec2 *points, int point_count, struct islc2d_vec2 trans) {
	for (int i = 0; i < point_count; i++) {
		points[i].x += trans.x;
		points[i].y += trans.y;
	}
}

void islc2d_rotate_polygon(struct islc2d_vec2 *points, int point_count, islc2d_float angle, struct islc2d_vec2 origin) {
	islc2d_float cosres = islc2d__cos(angle);
	islc2d_float sinres = islc2d__sin(angle);
	for (int i = 0; i < point_count; i++) {
		struct islc2d_vec2 v = points[i];
    v.x -= origin.x;
		v.y -= origin.y;
		points[i].x = v.x*cosres - v.y*sinres + origin.x;
		points[i].y = v.x*sinres + v.y*cosres + origin.y;
	}
}

bool islc2d_check_polygon_rect(struct islc2d_vec2 *points, int point_count, struct islc2d_rect rect, struct islc2d_vec3 *result) {
}

bool islc2d_check_polygon_circle(struct islc2d_vec *points, int point_count, struct islc2d_vec2 c, islc2d_float r, struct islc2d_vec3 *result) {
}

bool islc2d_check_polygon_rect_prepared(struct islc2d_vec2 *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect bbox, int point_count, struct islc2d_rect rect, struct islc2d_vec3 *result) {
}

bool islc2d_check_polygon_circle_prepared(struct islc2d_vec *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect bbox, int point_count, struct islc2d_vec2 c, islc2d_float r, struct islc2d_vec3 *result) {
}

bool islc2d_check_rect_circle(struct islc2d_rect rect, struct islc2d_vec2 c, islc2d_float r, struct islc2d_vec3 *result) {
}

bool islc2d_check_rects(struct islc2d_rect rect1, struct islc2d_rect rect2, struct islc2d_vec3 *result) {
	if ((rec1t.x > (rect2.x + rec2.width) || (rect1.x + rect1.width) < rect2.x) || (rect1.y > (rect2.y + rect2.height) || (rect1.y + rect1.height) < rect2.y)) return false;
	if (result) {
	}
	return true;
}

bool islc2d_check_circles(struct islc2d_vec2 c1, islc2d_float r1, struct islc2d_vec2 c2, islc2d_float r2, struct islc2d_vec3 *result) {
	islc2d_float dx = c1.x - c2.x, dy = c1.y - c2.y, r = r1 + r2;
	if (dx*dx + dy*dy > r*r) return false;
	if (result) {
	}
	return true;
}

void islc2d_scale_polygon(struct islc2d_vec2 *points, int point_count, struct islc2d_vec2 scale, struct islc2d_vec2 origin) {
}

void islc2d_prepare(struct islc2d_vec2 *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect *bbox, int point_count) {
	islc2d_float xmin = FLT_MAX, xmax = -FLT_MAX, ymin = FLT_MAX, ymax = -FLT_MAX;
	for (int i = 0; i < point_count; i++) {
		int np = (i == point_count - 1 ? 0 : i + 1);

		// generate the normal for the current edge
		struct islc2d_vec2 n = islc2d__normalize((struct islc2d_vec2) {-(points[i].y - points[np].y), points[i].x - points[np].x});

		// default min max
		islc2d_float min = FLT_MAX, max = -FLT_MAX;

		// project all vertices from poly1 onto axis
		for (int j = 0; j < point_count; j++) {
			islc2d_float dot = islc2d__dot_product(points[j], n);
			if (dot > max) max = dot;
			if (dot < min) min = dot;
		}

		ns[i] = n;
		minmax[i] = (struct islc2d_vec2) {min, max};
		if (points[i].x > xmax) xmax = points[i].x;
		if (points[i].x < xmin) xmin = points[i].x;
		if (points[i].y > ymax) ymax = points[i].y;
		if (points[i].y < ymin) ymin = points[i].y;
	}
	*bbox = (struct islc2d_rect) {xmin, ymin, xmax - xmin, ymax - ymin};
}

void islc2d_prepare_fixed_normals(struct islc2d_vec2 *points, struct islc2d_vec2 *ns, struct islc2d_vec2 *minmax, struct islc2d_rect *bbox, int point_count) {
	islc2d_float xmin = FLT_MAX, xmax = -FLT_MAX, ymin = FLT_MAX, ymax = -FLT_MAX;
	for (int i = 0; i < point_count; i++) {
		struct islc2d_vec2 n = ns[i];

		// default min max
		islc2d_float min = FLT_MAX, max = -FLT_MAX;

		// project all vertices from poly1 onto axis
		for (int j = 0; j < point_count; j++) {
			islc2d_float dot = islc2d__dot_product(points[j], n);
			if (dot > max) max = dot;
			if (dot < min) min = dot;
		}

		minmax[i] = (struct islc2d_vec2) {min, max};
		if (points[i].x > xmax) xmax = points[i].x;
		if (points[i].x < xmin) xmin = points[i].x;
		if (points[i].y > ymax) ymax = points[i].y;
		if (points[i].y < ymin) ymin = points[i].y;
	}
	*bbox = (struct islc2d_rect) {xmin, ymin, xmax - xmin, ymax - ymin};
}

bool islc2d_check_polygons(struct islc2d_vec2 *points1, int point_count1, struct islc2d_vec2 *points2, int point_count2, struct islc2d_vec3 *result) {
	islc2d_float interval;

	if (result) result->z = -FLT_MAX;

	// loop through the edges of Polygon 1
	for (int i = 0; i < point_count1; i++) {
		int np = (i == point_count1 - 1 ? 0 : i + 1);

		// generate the normal for the current edge
		struct islc2d_vec2 n = islc2d__normalize((struct islc2d_vec2) {-(points1[i].y - points1[np].y), points1[i].x - points1[np].x});

		// default min max
		islc2d_float min1 = FLT_MAX, min2 = FLT_MAX, max1 = -FLT_MAX, max2 = -FLT_MAX;

		// project all vertices from poly1 onto axis
		for (int j = 0; j < point_count1; j++) {
			islc2d_float dot = islc2d__dot_product(points1[j], n);
			if (dot > max1) max1 = dot;
			if (dot < min1) min1 = dot;
		}

		// project all vertices from poly2 onto axis
		for (int j = 0; j < point_count2; j++) {
			islc2d_float dot = islc2d__dot_product(points2[j], n);
			if (dot > max2) max2 = dot;
			if (dot < min2) min2 = dot;
		}

		// calculate the minimum translation vector should be negative
		if (min1 < min2) {
			interval = min2 - max1;
			n.x = -n.x;
			n.y = -n.y;
		} else {
			interval = min1 - max2;
		}

		// exit early if positive
		if (interval >= 0) return false;

		if (result && interval > result->z) {
			result->z = interval;
			result->x = n.x;
			result->y = n.y;
		}
	}

	// loop through the edges of Polygon 2
	for (int i = 0; i < point_count2; i++) {
		int np = (i == point_count2 - 1 ? 0 : i + 1);

		// generate the normal for the current edge
		struct islc2d_vec2 n = islc2d__normalize((struct islc2d_vec2) {-(points2[i].y - points2[np].y), points2[i].x - points2[np].x});

		// default min max
		islc2d_float min1 = FLT_MAX, min2 = FLT_MAX, max1 = -FLT_MAX, max2 = -FLT_MAX;

		//project all vertices from poly1 onto axis
		for (int j = 0; j < point_count1; j++) {
			islc2d_float dot = islc2d__dot_product(points1[j], n);
			if (dot > max1) max1 = dot;
			if (dot < min1) min1 = dot;
		}

		//project all vertices from poly2 onto axis
		for (int j = 0; j < point_count2; j++) {
			islc2d_float dot = islc2d__dot_product(points2[j], n);
			if (dot > max2) max2 = dot;
			if (dot < min2) min2 = dot;
		}

		//calculate the minimum translation vector should be negative
		if (min1 < min2) {
			interval = min2 - max1;
			n.x = -n.x;
			n.y = -n.y;
		} else {
			interval = min1 - max2;
		}

		//exit early if positive
		if (interval >= 0) return false;

		if (result && interval > result->z) {
			result->z = interval;
			result->x = n.x;
			result->y = n.y;
		}
	}
	return true;
}


bool islc2d_check_polygons_prepared(struct islc2d_vec2 *points1, struct islc2d_vec2 *ns1, struct islc2d_vec2 *minmax1, struct islc2d_rect bbox1, int point_count1, struct islc2d_vec2 *points2, struct islc2d_vec2 *ns2, struct islc2d_vec2 *minmax2, struct islc2d_rect bbox2, int point_count2, struct islc2d_vec3 *result) {
	islc2d_float interval;

	if (!CheckCollisionRecs(bbox1, bbox2)) return false;
	if (result) result->z = -FLT_MAX;

	// loop through the edges of Polygon 1
	for (int i = 0; i < point_count1; i++) {
		struct islc2d_vec2 n = ns1[i];

		// default min max
		islc2d_float min1 = minmax1[i].x, min2 = FLT_MAX, max1 = minmax1[i].y, max2 = -FLT_MAX;

		// project all vertices from poly2 onto axis
		for (int j = 0; j < point_count2; j++) {
			islc2d_float dot = islc2d__dot_product(points2[j], n);
			if (dot > max2) max2 = dot;
			if (dot < min2) min2 = dot;
		}

		// calculate the minimum translation vector should be negative
		if (min1 < min2) {
			interval = min2 - max1;
			n.x = -n.x;
			n.y = -n.y;
		} else {
			interval = min1 - max2;
		}

		// exit early if positive
		if (interval >= 0) return false;

		if (result && interval > result->z) {
			result->z = interval;
			result->x = n.x;
			result->y = n.y;
		}
	}

	// loop through the edges of Polygon 2
	for (int i = 0; i < point_count2; i++) {
		struct islc2d_vec2 n = ns2[i];

		// default min max
		islc2d_float min1 = FLT_MAX, min2 = minmax2[i].x, max1 = -FLT_MAX, max2 = minmax2[i].y;

		//project all vertices from poly1 onto axis
		for (int j = 0; j < point_count1; j++) {
			islc2d_float dot = islc2d__dot_product(points1[j], n);
			if (dot > max1) max1 = dot;
			if (dot < min1) min1 = dot;
		}

		//calculate the minimum translation vector should be negative
		if (min1 < min2) {
			interval = min2 - max1;
			n.x = -n.x;
			n.y = -n.y;
		} else {
			interval = min1 - max2;
		}

		//exit early if positive
		if (interval >= 0) return false;

		if (result && interval > result->z) {
			result->z = interval;
			result->x = n.x;
			result->y = n.y;
		}
	}
	return true;
}
/*
------------------------------------------------------------------------------
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2022 Ilya Kolbin
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
------------------------------------------------------------------------------
*/
#endif
