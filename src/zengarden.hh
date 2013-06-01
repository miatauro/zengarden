#ifndef _ZENGARDEN_H_
#define _ZENGARDEN_H_

#include <functional>
#include <initializer_list>
#include <ostream>
#include <vector>

namespace garden {
namespace geometry {
//optimization: for small types, don't use refs?
struct point {
  int x, y;
  point(std::initializer_list<int> init);
  point();
  bool operator==(const point& other) const;
  point operator+(const point& other) const;
  point operator-(const point& other) const;
  point operator*(const int other) const;
  friend std::ostream& operator<<(std::ostream& os, const point& p);
};
double distance(const point& p1, const point& p2);

//optimization: cache length?
struct line {
  point start, end;
  line();
  line(std::initializer_list<point> p);
  double slope() const;
  friend std::ostream& operator<<(std::ostream& os, const line& l);
  int dot(const line& o) const;
  double length() const;
  line normal() const;
  point mid() const;
  line operator+(const point& p) const;
  line operator-(const point& p) const;
  line operator*(const int n) const;
  line operator-(const line& l) const;
  line moveToStart(const point& p) const;
  bool operator==(const line& o) const;
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

enum class surface_type {REFLECT, DEFRACT};
struct surface {
  geometry::line pos;
  surface_type type = surface_type::REFLECT;
  surface(const geometry::line& l, const surface_type t) : pos(l), type(t) {}
};

struct ray {
  geometry::line pos;
  friend std::ostream& operator<<(std::ostream& os, const ray& r);
};

std::ostream& operator<<(std::ostream& os, const ray& r);

int scaleFactor(const int a, const int b);
int closer(const int a, const int b, const int x);
ray moveRayEndOutsideBox(const ray& r, const geometry::box& bounding);
ray generateRandomRayFromOrigin(const std::function<int()>& randomSource,
                                const geometry::box& bounding);
std::vector<ray> initialRays(const unsigned int rays, const geometry::box& bounding);

std::vector<std::pair<size_t, geometry::point> > collisions(const std::vector<surface>& sufaces,
                                                            const std::vector<ray>& rays);

ray reflect(const surface& surface, const ray& inputRay);
ray reflect(const surface& surface, const ray& inputRay,
            const geometry::point& collision);
ray refract(const surface& surface, const ray& inputRay,
            const geometry::point& collision);
ray bounce(const surface& surface, const ray& inputRay,
                        const geometry::point& collision);
std::vector<ray> bounceAll(const std::vector<surface>& surfaces, const std::vector<ray>& inputRays,
                        const std::vector<std::pair<size_t, geometry::point> >& collisions);
}
}
#endif /* _ZENGARDEN_H_ */
