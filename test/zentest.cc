#include "zengarden.hh"
#include <cassert>
#include <cstdlib>
#include <iostream>

#include "quickcheck/generate.hh"
#include "quickcheck/quickcheck.hh"

#include "gtest/gtest.h"

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
    generateParallelLines(n, a, b);
  }

  static void generateParallelLines(size_t n, line& a, line& b) {
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
    generateLinesThroughZero(n, a, b);
  }
  
  static void generateLineThroughZero(size_t n, line& l) {
    generate(n, l.start);
    while(l.start.x == 0 && l.start.y == 0)
      generate(n, l.start);

    l.end.x = (-l.start.x);
    l.end.y = (-l.start.y);
  }

  static void generateLinesThroughZero(size_t n, line& a, line& b) {
    generateLineThroughZero(n, a);
    generateLineThroughZero(n, b);
    if(a.slope() == b.slope())
      generateLinesThroughZero(n, a, b);
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

class PIntersectionAtPoint :
    public quickcheck::Property<point, line, line> {
 public:
  bool holdsFor(const point& p, const line& a, const line& b) {
    return p == intersection(a, b);
  }

  void generateInput(size_t n, point& p, line& a, line& b) {
    generateLinesThroughPoint(n, p, a, b);
  }

  static void generateLineThroughPoint(size_t n, const point& p, line& l) {
    generate(n, l.start);
    l.end.x = p.x + (p.x - l.start.x);
    l.end.y = p.y + (p.y - l.start.y);
  }

  static void generateLinesThroughPoint(size_t n, point& p, line& a, line& b) {
    generate(n, p);
    generateLineThroughPoint(n, p, a);
    generateLineThroughPoint(n, p, b);
  }
};


class PIntersectsParallel :
    public quickcheck::Property<line, line> {
 public:
  bool holdsFor(const line& a, const line& b) {
    return !segmentIntersects(a, b);
  }

  void generateInput(size_t n, line& a, line& b) {
    PIntersectionParallel::generateParallelLines(n, a, b);
  }
};


class PIntersectsAtZero :
    public quickcheck::Property<line, line> {
 public:
  bool holdsFor(const line& a, const line& b) {
    return segmentIntersects(a, b, {0, 0});
  }
  
  void generateInput(size_t n, line& a, line& b) {
    PIntersectionAtZero::generateLinesThroughZero(n, a, b);
  }
};

class PIntersects :
    public quickcheck::Property<line, line> {
 public:
  bool holdsFor(const line& a, const line& b) {
    point p = intersection(a, b);
    return (a.slope() == b.slope() ? !segmentIntersects(a, b) :
            (segmentIntersects(a, b) || !inBoundingBox(a.start, a.end, p)));
  }
  const std::string classify(const line& a, const line& b) {
    point p = intersection(a, b);
    if(a.slope() == b.slope())
      return "parallel";
    else if(!inBoundingBox(a.start, a.end, p))
      return "intersects, but not on a line";
    else
      return "intersects";
  }
};

class PIntersectsAtPoint :
    public quickcheck::Property<point, line, line> {
 public:
  bool holdsFor(const point& p, const line& a, const line& b) {
    return segmentIntersects(a, b, p);
  }
  void generateInput(size_t n, point& p, line& a, line& b) {
    PIntersectionAtPoint::generateLinesThroughPoint(n, p, a, b);
  }
};

class PAngleBetweenQuarter :
    public quickcheck::Property<line, line> {
 public:
  bool holdsFor(const line& a, const line& b) {
    double angle = angleBetween(a, b);
    //
    if(a.end.x >= 0) {
      if(b.end.x >= 0) {
        return 0.0 <= angle && angle <= M_PI_2;
      }
      else {
        return M_PI_2 <= angle && angle <= M_PI;
      }
    }
    else {
      if(b.end.x < 0) {
        return 0.0 <= angle && angle <= M_PI_2;
      }
      else {
        return M_PI_2 <= angle && angle <= M_PI;
      }
    }
  }
  
  void generateInput(size_t n, line& a, line& b) {
    a.start = {0, 0};
    a.end.y = 0;
    while(a.end.x == 0)
      quickcheck::generate(n, a.end.x);

    b.start = {0, 0};
    generate(n, b.end);
  }
};

// class PAngleBetween :
//     public quickcheck::Property<double, line, line> {
//  public:
//   void holdsFor(const double& angle, const line& a, const line& b) {
//     return angleBetween(a, b) == angle;
//   }
//   void generateInput(size_t n, double& angle, line& a, line& b) {
//     angle = drand48() * M_PI;
//     generate(n, a);
    
//   }
// };

#define QC_TEST(QC_PROP_CLASS, QC_TEST_CLASS, QC_TEST, QC_MESSAGE) TEST(QC_TEST_CLASS, QC_TEST) { \
  ASSERT_TRUE(quickcheck::check<QC_PROP_CLASS>(QC_MESSAGE)); \
}

QC_TEST(PInBoundingBox, BoundingBoxTest, InBox, "In bounding box");
QC_TEST(PInBoundingBoxB, BoundingBoxTest, BoxInBox, "In bounding box");

QC_TEST(PIntersectionParallel, IntersectionTest, ParallelIntersection,
        "Intersection of parallel lines.");
QC_TEST(PIntersectionAtZero, IntersectionTest, IntersectionAtZero,
        "Intersections of lines at zero.");
QC_TEST(PHasIntersection, IntersectionTest, HasIntersection, "Has intersections.");
QC_TEST(PIntersectionAtPoint, IntersectionTest, IntersectsAtPoint, "Intersects at point.");

QC_TEST(PIntersectsParallel, SegmentIntersectionTest, ParallelIntersection,
        "Segment intersection of parallel lines.");
QC_TEST(PIntersectsAtZero, SegmentIntersectionTest, IntersectionAtZero,
        "Segment intersections of lines at zero.");
QC_TEST(PIntersects, SegmentIntersectionTest, HasIntersection, "Has segment intersections.");
QC_TEST(PIntersectsAtPoint, SegmentIntersectionTest, IntersectsAtPoint,
        "Segment intersects at point.");

QC_TEST(PAngleBetweenQuarter, AngleBetweenTest, TwoLines, "Angle between two lines.");

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}
