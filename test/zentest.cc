#include "zengarden.hh"
#include <cassert>
#include <cstdlib>
#include <iostream>

#include "quickcheck/generate.hh"
#include "quickcheck/quickcheck.hh"

#include "gtest/gtest.h"

using namespace garden::geometry;
using namespace garden::rays;

namespace garden { namespace geometry {

void generate(size_t n, point& p) {
  quickcheck::generate(n, p.x);
  quickcheck::generate(n, p.y);
}

void generate(size_t n, line& l) {
  generate(n, l.start);
  generate(n, l.end);
}

}
namespace rays {
void generate(size_t n, ray& r) {
  generate(n, r.pos);
}
}
}

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
            (segmentIntersects(a, b) ||
             !(inBoundingBox(a.start, a.end, p) && inBoundingBox(b.start, b.end, p))));
  }
  const std::string classify(const line& a, const line& b) {
    point p = intersection(a, b);
    if(a.slope() == b.slope())
      return "parallel";
    else if(!(inBoundingBox(a.start, a.end, p) && inBoundingBox(b.start, b.end, p)))
      return "intersects, but not on both lines";
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

class PPointAdd : public quickcheck::Property<point, point> {
  bool holdsFor(const point& a, const point& b) {
    point sum = a + b;
    return sum.x == (a.x + b.x) && sum.y == (a.y + b.y);
  }
};

class PPointSub : public quickcheck::Property<point, point> {
  bool holdsFor(const point& a, const point& b) {
    point sum = a - b;
    return sum.x == (a.x - b.x) && sum.y == (a.y - b.y);
  }
};

class PPointMul : public quickcheck::Property<point, int> {
  bool holdsFor(const point& a, const int& b) {
    point p = a * b;
    return p.x == (a.x * b) && p.y == (a.y * b);
  }
};

class PPointEq : public quickcheck::Property<point, point> {
  bool holdsFor(const point& a, const point& b) {
    return (a == b) == ((a.x == b.x && a.y == b.y));
  }
};

class PPointOut : public quickcheck::Property<point> {
  bool holdsFor(const point& p) {
    std::stringstream check;
    check << p;
    std::stringstream gold;
    gold << "(" << p.x << ", " << p.y << ")";
    return check.str() == gold.str();
  }
};


class PBoxOut : public quickcheck::Property<box> {
  bool holdsFor(const box& b) {
    std::stringstream check;
    check << b;
    std::stringstream gold;
    gold << "(" << b.first << " " << b.second << ")";
    return check.str() == gold.str();
  }
};

class PLineOut : public quickcheck::Property<line> {
  bool holdsFor(const line& l) {
    std::stringstream check;
    check << l;
    std::stringstream gold;
    gold << l.start << " -> " << l.end;
    return check.str() == gold.str();
  }
};

class PLineSlope : public quickcheck::Property<line> {
  bool holdsFor(const line& l) {
    if(l.end.x - l.start.x == 0)
      return true;
    EXPECT_DOUBLE_EQ(static_cast<double>(l.end.y - l.start.y) /
                     static_cast<double>(l.end.x - l.start.x),
                     l.slope());
    return true;
  }
};

class PLineDot : public quickcheck::Property<line, line> {
  bool holdsFor(const line& l1, const line& l2) {
    return l1.dot(l2) ==
        (((l1.end.x - l1.start.x) * (l2.end.x - l2.start.x)) + 
         ((l1.end.y - l1.start.y) * (l2.end.y - l2.start.y)));
  }
};

class PLineLength : public quickcheck::Property<line>{
  bool holdsFor(const line& l) {
    return l.length() == std::sqrt(std::pow(l.end.x - l.start.x, 2) +
                                   std::pow(l.end.y - l.start.y, 2));
  }
};

//TODO: figure out why the commented bit isn't working right
class PLineMid : public quickcheck::Property<line, point> {
  bool holdsFor(const line& l, const point& p) {
    point m = l.mid();
    return p == m;
    // EXPECT_EQ(distance(m, l.start), distance(m, l.end)) << "Equidistant from endpoints.";
    // EXPECT_LE(std::min(l.start.x, l.end.x), m.x) << "Greater than least x.";
    // EXPECT_LE(m.x, std::max(l.start.x, l.end.x)) << "Less than greatest x.";
    // EXPECT_LE(std::min(l.start.y, l.end.y), m.y) << "Greater than least y.";
    // EXPECT_LE(m.y, std::max(l.start.y, l.end.y)) << "Less than greatest y.";
    // EXPECT_DOUBLE_EQ((m.y - l.start.y), (l.slope() * (m.x - l.start.x))) << "Point test";

    // return distance(m, l.start) == distance(m, l.end) &&
    //     std::min(l.start.x, l.end.x) <= m.x &&
    //     m.x <= std::max(l.start.x, l.end.x) && 
    //     std::min(l.start.y, l.end.y) <= m.y &&
    //        m.y <= std::max(l.start.y, l.end.y);/* &&
    //        std::floor((m.y - l.start.y) == std::floor(l.slope() * (m.x - l.start.x)));*/
  }
  void generateInput(size_t n, line& l, point& p) {
    generateLineWithMidpoint(n, l, p);
  }
 public:
  static void generateLineWithMidpoint(size_t n, line& l, point& p) {
    generate(n, p);
    int xdiffs, ydiffs;
    quickcheck::generate(n, xdiffs);
    quickcheck::generate(n, ydiffs);
    l.start.x = p.x + xdiffs;
    l.end.x = p.x - xdiffs;
    l.start.y = p.y + ydiffs;
    l.end.y = p.y - ydiffs;
  }
};

class PLineNormal : public quickcheck::Property<line> {
  bool holdsFor(const line& l) {
    line n = l.normal();
    return (l.dot(n) == 0) && (n.dot(l) == 0);
  }
  void generateInput(size_t n, line& l) {
    point p;
    PLineMid::generateLineWithMidpoint(n, l, p);
  }
};

class PLineAddPoint : public quickcheck::Property<line, point> {
  bool holdsFor(const line& l, const point& p) {
    line l2({{l.start.x + p.x, l.start.y + p.y},
        {l.end.x + p.x, l.end.y + p.y}});
    return (l + p) == l2;
  }
};

class PLineSubPoint : public quickcheck::Property<line, point> {
  bool holdsFor(const line& l, const point& p) {
    line l2({{l.start.x - p.x, l.start.y - p.y},
        {l.end.x - p.x, l.end.y - p.y}});
    return (l - p) == l2;
  }
};

class PLineMulInt : public quickcheck::Property<line, int> {
  bool holdsFor(const line& l, const int& n) {
    line m = l * n;
    return (m.start == l.start) && (m.end == l.end * n);
  }
};

class PLineSubLine : public quickcheck::Property<line, line> {
  bool holdsFor(const line& l1, const line& l2) {
    line s = l1 - l2;
    return s.start == l1.start && s.end == (l1.end - l2.end);
  }
};

class PLineMoveToStart : public quickcheck::Property<line, point> {
  bool holdsFor(const line& l, const point& p) {
    line moved = l.moveToStart(p);
    return moved.start == p && moved.length() == l.length() &&
        (std::isnan(l.slope()) ? std::isnan(moved.slope()) :  moved.slope() == l.slope());
  }
};

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/variate_generator.hpp>

class PTestGenerateRandomFromOrigin : public quickcheck::Property<ray> {
 public:
PTestGenerateRandomFromOrigin() : Property<ray>(), m_r(0), gen(std::time(nullptr)), dist(-50, 50),
                                  generator(gen, dist), fn(generator) {
    m_bounding.first = {-50, -50};
    m_bounding.second = {50, 50};
  }
  void generateInput(size_t n, ray& r) {
    r = generateRandomRayFromOrigin(fn, m_bounding);
  }
  bool holdsFor(const ray& r) {
    return r.pos.start == point({0, 0}) &&
        ((r.pos.end.x <= m_bounding.first.x ||
          r.pos.end.x >= m_bounding.second.x) ||
         (r.pos.end.y <= m_bounding.first.y ||
          r.pos.end.y >= m_bounding.second.y));
  }

  unsigned int m_r;
  box m_bounding;
  boost::random::mt19937 gen;
  boost::random::uniform_int_distribution<int> dist;
  boost::random::variate_generator<boost::random::mt19937,
                                   boost::random::uniform_int_distribution<int>> generator;
  std::function<int()> fn;
};



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

QC_TEST(PPointAdd, Point, Add, "Sum of points.");
QC_TEST(PPointSub, Point, Subtract, "Difference of points.");
QC_TEST(PPointMul, Point, Multiply, "Multiplying a point by an integer.");
QC_TEST(PPointEq, Point, Equal, "Point equality.");
QC_TEST(PPointOut, Point, Output, "Point output.");

QC_TEST(PBoxOut, Box, Output, "Box output.");

QC_TEST(PLineOut, Line, Output, "Line output.");
QC_TEST(PLineSlope, Line, Slope, "Line slope.");
QC_TEST(PLineDot, Line, Dot, "Line dot product.");
QC_TEST(PLineLength, Line, Length, "Line length.");
QC_TEST(PLineNormal, Line, Normal, "Line normal.");
QC_TEST(PLineMid, Line, Mid, "Line midpoint.");
QC_TEST(PLineAddPoint, Line, AddPoint, "Adding point to line.");
QC_TEST(PLineSubPoint, Line, SubPoint, "Subtracting point from line.");
QC_TEST(PLineMulInt, Line, MulInt, "Multiplying line by an int.");
QC_TEST(PLineSubLine, Line, SubLine, "Subtracting one line from another.");
QC_TEST(PLineMoveToStart, Line, MoveToStart, "Moving a line to start at a point.");

QC_TEST(PTestGenerateRandomFromOrigin, Rays, GenerateRandomFromOrigin,
        "Random ray generation.");

using namespace garden::rays;

TEST(AngleBetween, Angles) {
  line base({{0, 0}, {3, 0}});
  ASSERT_DOUBLE_EQ(angleBetween(base, line({{1, 0}, {1, 3}})), M_PI_2);
  ASSERT_DOUBLE_EQ(angleBetween(base, line({{1, 0}, {1, -3}})), M_PI_2);

  ASSERT_DOUBLE_EQ(angleBetween(base, line({{1, 0}, {3, 2}})), M_PI_4);
  ASSERT_DOUBLE_EQ(angleBetween(base, line({{1, 0}, {3, -2}})), M_PI_4);

  ASSERT_DOUBLE_EQ(angleBetween(base, line({{1, 0}, {-1, 2}})), 3.0 * M_PI_4);
  ASSERT_DOUBLE_EQ(angleBetween(base, line({{1, 0}, {-1, -2}})), 3.0 * M_PI_4);

}

TEST(CollisionDetection, NoCollision) {
  surface s({{0, 0}, {3, 3}}, surface_type::REFLECT);
  ray r({{{-4, -4}, {-2, -2}}});
  auto check = collisions({s}, {r});
  ASSERT_EQ(check.size(), 1U) << "Generated fewer collisions than rays.";
  ASSERT_EQ(check[0].first, std::numeric_limits<size_t>::max()) << "Found a collision where there should be none at " << check[0].second;
}

TEST(CollisionDetection, NoCollisionOnePast) {
  surface s({{0, 0}, {3, 3}}, surface_type::REFLECT);
  ray r({{{-4, 4}, {4, 4}}});
  auto check = collisions({s}, {r});
  ASSERT_EQ(check.size(), 1U) << "Generated fewer collisions than rays.";
  ASSERT_EQ(check[0].first, std::numeric_limits<size_t>::max()) << "Found a collision where there should be none at " << check[0].second;
}

TEST(CollisionDetection, CollisionAtEnd) {
  surface s({{0, 0}, {3, 3}}, surface_type::REFLECT);
  ray r1({{{-4, 3}, {3, 3}}});
  ray r2({{{-4, 0}, {4, 0}}});
  std::vector<ray> rays = {r1, r2};
  auto check = collisions({s}, rays);
  ASSERT_EQ(check.size(), rays.size()) << "Generated fewer collisions than rays.";
  ASSERT_EQ(check[0].first, 0U) << "Expected a collision with the only surface.";
  ASSERT_EQ(check[0].second, s.pos.end);

  ASSERT_EQ(check[1].first, 0U) << "Expected a collision with the only surface.";
  ASSERT_EQ(check[1].second, s.pos.start);
}

TEST(CollisionDetect, MultipleCollisions) {
  surface s0({{2, 0}, {5, 3}}, surface_type::REFLECT);
  surface s1({{0, 0}, {3, 3}}, surface_type::REFLECT);
  ray r({{{0, 1}, {5, 1}}});
  auto check = collisions({s0, s1}, {r});
  ASSERT_EQ(check.size(), 1U) << "Generated fewer collisions than rays.";
  ASSERT_EQ(check[0].first, 1U) << "Expected a collision with the closer surface.";
  ASSERT_EQ(check[0].second, point({1, 1}));
}

TEST(Line, DotSelf) {
  line l1({{0, 0}, {0, 3}});
  ASSERT_DOUBLE_EQ(l1.dot(l1), l1.length() * l1.length());
}
TEST(Line, DotPerpendicular) {
  line l1({{0, 0}, {0, 3}});
  line l2({{0, 0}, {3, 0}});
  line l3({{0, 0}, {-3, 0}});
  ASSERT_EQ(l1.dot(l2), 0);
  ASSERT_EQ(l2.dot(l1), 0);
}

TEST(Line, DotTranslate) {
  line l1({{0, 0}, {0, 3}});
  line l2({{0, 0}, {3, 0}});
  point translate({4, 4});
  ASSERT_EQ((l1 + translate).dot(l2 + translate), 0);
}

TEST(Line, DotOpposite) {
  line l1({{0, 0}, {0, 3}});
  line l2({{0, 0}, {0, -3}});
  ASSERT_DOUBLE_EQ(l1.dot(l2), -(l1.length() * l2.length()));
}

TEST(Line, MoveToStartParticular) {
  line l{{-4, -2}, {-4, -3}};
  point p{-6, -2};
  line moved = l.moveToStart(p);
  
  ASSERT_EQ(p, moved.start);
  ASSERT_EQ(l.length(), moved.length());
  ASSERT_EQ(std::isnan(l.slope()), std::isnan(moved.slope()));
}

TEST(Rays, ScaleFactor) {
  ASSERT_EQ(scaleFactor(1, 1), 1);
  ASSERT_EQ(scaleFactor(4, 2), 2);
  ASSERT_EQ(scaleFactor(5, 2), 3);
  ASSERT_EQ(2, scaleFactor(-4, -2));
  ASSERT_EQ(3, scaleFactor(-5, -2));
}

TEST(Rays, Closer) {
  ASSERT_EQ(closer(1, 3, 0), 1);
  ASSERT_EQ(closer(3, 4, 0), 3);
  ASSERT_EQ(closer(-3, 3, 0), 3);
  ASSERT_EQ(closer(-3, 4, 0), -3);
}

TEST(Rays, InitialRays) {
  std::vector<ray> rays = initialRays(300, {{-50, -50}, {50, 50}});
  ASSERT_EQ(300U, rays.size());
}

// TEST(Surface, ReflectOnly) {
//   surface s{{{-3, 3}, {3, 3}}, surface_type::REFLECT};
//   ray r1{{-3, -3}, {3, 3}};
//   ray reflected = reflect(s, r1);
// }

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}
