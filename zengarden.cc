//#include <math>
//#include <cstdlib>
#include <limits>
#include <utility>


#include "zengarden.hh"
namespace garden {
namespace geometry {

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
  int l1ydiff = l1.end.y - l1.end.y;
  int l2xdiff = l2.start.x - l2.end.x;
  int l2ydiff = l2.end.y - l2.end.y;
  
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
  return std::min(p1.x, p2.x) <= intersection.x && intersection.x <= std::min(p1.x, p2.x) &&
      std::min(p1.y, p2.y) <= intersection.y && intersection.y <= std::min(p1.y, p2.y);
}
//given two line segments l1 and l2 and a point of intersection between them,
//returns true if the intersection point is actually on one of the two lines.
bool segmentIntersects(const line& l1, const line& l2, const point& intersection) {
  return inBoundingBox(l1.start, l1.end, intersection);
}

bool segmentIntersects(const line& l1, const line& l2) {
  return segmentIntersects(l1, l2, intersection);
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

ray generateRandomRayFromOrigin(drand48_data* d, const box& bounding) {
  long int rx, ry;
  mrand48_r(&drand48_data, &rx);
  mrand48_r(&drand48_data, &ry);
    
  if(inBoundingBox({rx, ry}, bounding)) {
    int factor = std::max(scaleFactor(closer(bounding.first.x, bounding.second.x, rx), rx),
                          scaleFactor(closer(bounding.first.y, bounding.second.y, ry), ry));
    
    return {{0, 0}, {rx * factor, ry * factor}};
  }
  return {{0, 0}, {rx, ry}};
}
//generate a vector of random rays starting at (0, 0) and ending at a random point
//outside the bounding box
std::vector<ray> initialRays(const unsigned int rays, const box& bounding) {
  std::vector<ray> retval;
  retval.reserve(rays);
  
  drand48_data d;
  
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
  retval.reserve(rays);
  for(auto& r : rays) {
    bool collided = false;
    for(auto sIt = surfaces.begin(); sIt != surfaces.end() && !collided; ++sIt) {
      point p = intersection(r.pos, sIt->pos);
      collided = segmentIntersects(r.pos, sIt->.pos, p);
      if(collided)
        retval.push_back({std::distance(surfaces.begin(), sIt), p});
    }
    if(!collided)
      retval.push_back(std::numeric_limits<size_t>::max(), {0, 0});
  }
  return retval;
}

}
}
