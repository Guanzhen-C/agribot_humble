#include "../plugins/engagement.hpp"
#include <cassert>
#include <limits>
int main()
{
    using agribot_hitch::engaged;
    assert(engaged(.3, .01, 0, 0, -28, 28, -8, 8));
    assert(engaged(.3, -.10, 0, 0, -28, 28, -8, 8));
    assert(!engaged(0., .01, 0, 0, -28, 28, -8, 8));
    assert(!engaged(.3, .5, 0, 0, -28, 28, -8, 8));
    assert(!engaged(.3, .01, 29, 0, -28, 28, -8, 8));
    assert(!engaged(.3, -.3, 0, 0, -28, 28, -8, 8));
    assert(!engaged(.3, std::numeric_limits<double>::quiet_NaN(), 0, 0, -28, 28, -8, 8));
}
