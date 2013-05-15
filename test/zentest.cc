#include "zengarden.hh"

#include "quickcheck/quickcheck.hh"

using namespace garden::geometry;

class PInBoundingBox : 
    public quickcheck::Property<point, point, point> {
 public:
  bool holdsFor(const point& p1, const point& p2, const point& p3) {
    bool retval = inBoundingBox(p1, p2, p3);
  }

};

int main(void) {

  
}
