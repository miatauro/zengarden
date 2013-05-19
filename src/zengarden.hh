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

struct line {
point start, end;
line();
line(std::initializer_list<point> p);
double slope() const;
friend std::ostream& operator<<(std::ostream& os, const line& l);
};
typedef std::pair<point, point> box;

point intersection(const line& l1, const line& l2);
bool inBoundingBox(const point&p1, const point&p2, const point& test);
bool inBoundingBox(const box& b, const point& test);
bool segmentIntersects(const line& l1, const line& l2, const point& intersection);
bool segmentIntersects(const line& l1, const line& l2);


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

}
}
#endif /* _ZENGARDEN_H_ */
