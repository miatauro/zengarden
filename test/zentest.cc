#include "zengarden.hh"
#include <cassert>
#include <cstdlib>
#include <iostream>

#include "quickcheck/generate.hh"
#include "quickcheck/quickcheck.hh"

using namespace garden::geometry;

namespace garden { namespace geometry {

void generate(size_t n, point& p) {
  quickcheck::generate(n, p.x);
  quickcheck::generate(n, p.y);
}

void generate(size_t n, line& l) {
  generate(n, l.start);
  generate(n, l.end);
}

}}

class PInBoundingBox : 
    public quickcheck::Property<point, point, point> {
 public:
  bool holdsFor(const point& p1, const point& p2, const point& p3) {
    return inBoundingBox(p1, p2, p3) ==
      ((std::min(p1.x, p2.x) <= p3.x && p3.x <= std::max(p1.x, p2.x)) &&
       (std::min(p1.y, p2.y) <= p3.y && p3.y <= std::max(p1.y, p2.y)));
  }
};

class PInBoundingBoxB : 
    public quickcheck::Property<box, point> {
 public:
  bool holdsFor(const box& b, const point& p) {
    return inBoundingBox(b, p) ==
      ((std::min(b.first.x, b.second.x) <= p.x && p.x <= std::max(b.first.x, b.second.x)) &&
       (std::min(b.first.y, b.second.y) <= p.y && p.y <= std::max(b.first.y, b.second.y)));
  }
};

class PIntersectionParallel :
    public quickcheck::Property<line, line> {
 public:
  bool holdsFor(const line& a, const line& b) {
    point p = intersection(a, b);
    return p.x == std::numeric_limits<int>::min() && p.y == std::numeric_limits<int>::min();
  }

  void generateInput(size_t n, line& a, line& b) {
    generate(n, a);
    generate(n, b.start);
    b.end.x = ((a.end.x - a.start.x) + b.start.x);
    b.end.y = ((a.end.y - a.start.y) + b.start.y);
  }
};


class PIntersectionAtZero :
    public quickcheck::Property<line, line> {
 public:
  bool holdsFor(const line& a, const line& b) {
    point p = intersection(a, b);
    return p.x == 0 && p.y == 0;
  }
  
  void generateInput(size_t n, line& a, line& b) {
    generateLineThroughZero(n, a);
    generateLineThroughZero(n, b);
    if(a.slope() == b.slope())
      generateInput(n, a, b);
  }
  
  void generateLineThroughZero(size_t n, line& l) {
    generate(n, l.start);
    while(l.start.x == 0 && l.start.y == 0)
      generate(n, l.start);

    l.end.x = (-l.start.x);
    l.end.y = (-l.start.y);
  }
};

class PHasIntersection :
    public quickcheck::Property<line, line> {
 public:
  bool holdsFor(const line& a, const line& b) {
    point p = intersection(a, b);
    return (a.slope() == b.slope() ?
            p.x == std::numeric_limits<int>::min() && p.y == std::numeric_limits<int>::min() :
            p.x != std::numeric_limits<int>::min() || p.y != std::numeric_limits<int>::min());
  }
  const std::string classify(const line& a, const line& b) {
    if(a.slope() == b.slope())
      return "parallel";
    else
      return "intersects somewhere";
  }
};

class PIntersectsAtPoint :
    public quickcheck::Property<point, line, line> {
 public:
  bool holdsFor(const point& p, const line& a, const line& b) {
    return p == intersection(a, b);
  }
  void generateInput(size_t n, point& p, line& a, line& b) {
    generate(n, p);
    generateLineThroughPoint(n, p, a);
    generateLineThroughPoint(n, p, b);
  }

  void generateLineThroughPoint(size_t n, const point& p, line& l) {
    generate(n, l.start);
    l.end.x = p.x + (p.x - l.start.x);
    l.end.y = p.y + (p.y - l.start.y);
  }
};

int main(void) {
  assert(quickcheck::check<PInBoundingBox>("In bounding box"));
  assert(quickcheck::check<PInBoundingBoxB>("In bounding box"));
  assert(quickcheck::check<PIntersectionParallel>("Intersection of parallel lines."));
  assert(quickcheck::check<PIntersectionAtZero>("Intersections of lines at zero."));
  assert(quickcheck::check<PHasIntersection>("Has intersections."));
  assert(quickcheck::check<PIntersectsAtPoint>("Intersects at point."));
  return 0;
}
