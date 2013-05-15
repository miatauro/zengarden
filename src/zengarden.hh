#ifndef _ZENGARDEN_H_
#define _ZENGARDEN_H_

#include <initializer_list>
#include <vector>

namespace garden {
namespace geometry {

struct point {
  int x, y;
  point(std::initializer_list<int> init);
};
struct line {point start, end;};
typedef std::pair<point, point> box;

point intersection(const line& l1, const line& l2);
bool inBoundingBox(const point&p1, const point&p2, const point& test);
bool inBoundingBox(const box& b, const point& test);
bool segmentIntersects(const line& l1, const line& l2, const point& intersection);
bool segmentIntersects(const line& l1, const line& l2);
}

namespace rays {
struct surface {
  geometry::line pos;
};

struct ray {
  geometry::line pos;
};

std::vector<ray> initialRays(const unsigned int rays, const geometry::box& bounding);

}
}
#endif /* _ZENGARDEN_H_ */
