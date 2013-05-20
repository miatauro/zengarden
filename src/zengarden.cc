#include <cassert>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

#include "zengarden.hh"

namespace garden {
namespace geometry {

point::point(std::initializer_list<int> init) : x(*(init.begin())), y(*(init.begin() + 1)) {}
point::point() : x(0), y(0) {}
bool point::operator==(const point& other) const {
  return x == other.x && y == other.y;
}


std::ostream& operator<<(std::ostream& os, const point& p) {
  os << "(" << p.x << ", " << p.y << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const box& b) {
  os << "(" << b.first << " " << b.second << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const line& l) {
  os << l.start << " -> " << l.end;
  return os;
}

line::line() : start(), end() {}

line::line(std::initializer_list<point> points)
    : start(*(points.begin())), end(*(points.begin() + 1)) {}

//use float?
double line::slope() const {
  return (end.x == start.x ? std::numeric_limits<double>::quiet_NaN() :
          static_cast<double>(end.y - start.y) / static_cast<double>(end.x - start.x));
}

int line::dot(const line& o) const {
  return (end.x - start.x) * (o.end.x - o.start.x) +
      (end.y - start.y) * (o.end.y - o.end.y);
}

double line::length() const {
return  std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
}

//two lines L1 ((x_1, y_1) -> (x_2, y_2)) and L2 ((x_3, y_3) -> (x_4, y_4))

//P_x = ((x_1 y_2) - (y_1 x_2))(x_3 - x_4) - (x_1 - x_2)((x_3 y_4- (y_3 x_4))
//      ---------------------------------------------------------------------
//         (x_1 - x_2)(y_3 - y_4) - (y_1 - y_2)(x_3 - x_4)

//P_y = ((x_1 y_2) - (y_1 x_2))(y_3 - y_4) - (y_1 - y_2)((x_3 y_4- (y_3 x_4))
//      ---------------------------------------------------------------------
//         (x_1 - x_2)(y_3 - y_4) - (y_1 - y_2)(x_3 - x_4)

//given two lines l1 and l2, returns the point at which they intersect or, (-min, -min)
//if they are parallel.  note that the returned point may not be on either line segment
//as it determines the intersection point between the two infinite lines that contain l1 and l2
point intersection(const line& l1, const line& l2) {
  int l1xdiff = l1.start.x - l1.end.x;
  int l1ydiff = l1.start.y - l1.end.y;
  int l2xdiff = l2.start.x - l2.end.x;
  int l2ydiff = l2.start.y - l2.end.y;
  
  int denom = (l1xdiff * l2ydiff) - (l1ydiff * l2xdiff);

  if(denom == 0) {
    return {std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
  }

  int a1 = (l1.start.x * l1.end.y) - (l1.start.y * l1.end.x);
  int a2 = (l2.start.x * l2.end.y) - (l2.start.y * l2.end.x);

  return {((a1 * l2xdiff) - (l1xdiff * a2)) / denom, ((a1 * l2ydiff) - (l1ydiff * a2)) / denom};

}

//determine if a test point is in a bounding box defined by two other points
bool inBoundingBox(const box& b, const point& test) {
  return inBoundingBox(b.first, b.second, test);
}
bool inBoundingBox(const point& p1, const point& p2, const point& test) {
  return std::min(p1.x, p2.x) <= test.x && test.x <= std::max(p1.x, p2.x) &&
      std::min(p1.y, p2.y) <= test.y && test.y <= std::max(p1.y, p2.y);
}
//given two line segments l1 and l2 and a point of intersection between them,
//returns true if the intersection point is actually on one of the two lines.
bool segmentIntersects(const line& l1, const line& l2, const point& intersection) {
  return inBoundingBox(l1.start, l1.end, intersection);
}

bool segmentIntersects(const line& l1, const line& l2) {
  return segmentIntersects(l1, l2, intersection(l1, l2));
}

double angleBetween(const line& l1, const line& l2) {
  double retval = std::acos(l1.dot(l2) / (l1.length() * l2.length()));
  return retval;
}

}

namespace rays {
using namespace geometry;

int scaleFactor(const int a, const int b) {
  assert((a >= 0 && b >= 0) || (a <= 0 && b <= 0));
  return static_cast<int>(std::ceil(static_cast<double>(a) / static_cast<double>(b)));
}

int closer(const int a, const int b, const int x) {
  return (std::abs(a - x) < std::abs(b - x)) ? a : b;
}

ray generateRandomRayFromOrigin(unsigned int* d, const box& bounding) {
  int rx = rand_r(d);
  int ry = rand_r(d);
    
  if(inBoundingBox(bounding, {static_cast<int>(rx), static_cast<int>(ry)})) {
    int factor = std::max(scaleFactor(closer(bounding.first.x, bounding.second.x, rx), rx),
                          scaleFactor(closer(bounding.first.y, bounding.second.y, ry), ry));
    
    return {{{0, 0}, {rx * factor, ry * factor}}};
  }
  return {{{0, 0}, {rx, ry}}};
}
//generate a vector of random rays starting at (0, 0) and ending at a random point
//outside the bounding box
std::vector<ray> initialRays(const unsigned int rays, const box& bounding) {
  std::vector<ray> retval;
  retval.reserve(rays);
  
  unsigned int d;
  
  for(unsigned int i = 0; i < rays; ++i)
    retval.push_back(generateRandomRayFromOrigin(&d, bounding));

  return retval;
}

//generate a vector of surface indices indicating which surfaces bounce which rays
//-1 indicates that a ray intersects no surface
//and at which point they intersect
std::vector<std::pair<size_t, point> > collisions(const std::vector<surface>& surfaces,
                                               const std::vector<ray>& rays) {
  std::vector<std::pair<size_t, point> > retval;
  retval.reserve(rays.size());
  for(auto rIt = rays.begin(); rIt != rays.end(); ++rIt) {
    bool collided = false;
    for(auto sIt = surfaces.begin(); sIt != surfaces.end() && !collided; ++sIt) {
      point p = intersection(rIt->pos, sIt->pos);
      collided = segmentIntersects(rIt->pos, sIt->pos, p);
      if(collided)
        retval.push_back({std::distance(surfaces.begin(), sIt), p});
    }
    if(!collided)
      retval.push_back({std::numeric_limits<size_t>::max(), {0, 0}});
  }
  return retval;
}

}
}
