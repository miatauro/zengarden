#ifndef _ZENGARDEN_H_
#define _ZENGARDEN_H_

#include <initializer_list>
#include <ostream>
#include <vector>

namespace garden {
namespace geometry {

struct point {
  int x, y;
  point(std::initializer_list<int> init);
  point();
  bool operator==(const point& other) const;
  friend std::ostream& operator<<(std::ostream& os, const point& p);
};

//optimization: cache length?
struct line {
  point start, end;
  line();
  line(std::initializer_list<point> p);
  double slope() const;
  friend std::ostream& operator<<(std::ostream& os, const line& l);
  int dot(const line& o) const;
  double length() const;
};
typedef std::pair<point, point> box;

point intersection(const line& l1, const line& l2);
bool inBoundingBox(const point&p1, const point&p2, const point& test);
bool inBoundingBox(const box& b, const point& test);
bool segmentIntersects(const line& l1, const line& l2, const point& intersection);
bool segmentIntersects(const line& l1, const line& l2);
double angleBetween(const line& l1, const line& l2);

std::ostream& operator<<(std::ostream& os, const point& p);
std::ostream& operator<<(std::ostream& os, const box& b);
std::ostream& operator<<(std::ostream& os, const line& l);
}

namespace rays {

struct surface {
  geometry::line pos;
};

struct ray {
  geometry::line pos;
};

std::vector<ray> initialRays(const unsigned int rays, const geometry::box& bounding);

std::vector<std::pair<size_t, geometry::point> > collisions(const std::vector<surface>& sufaces,
                                                            const std::vector<ray>& rays);

std::vector<ray> bounce(const std::vector<surface>& surfaces, const std::vector<ray>& inputRays);
}
}
#endif /* _ZENGARDEN_H_ */
