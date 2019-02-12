#include "sincos.h"
#include <cmath>
void test_sincos() {
    float x[] = {-100, 10, .2, 0, 1.1, 10, 100};

    for(int i=0; i<sizeof(x)/sizeof(float); i++) {
        Sincos sc = sincos(x[i]);
        assert(sc.cos == std::cos(x[i]));
        assert(sc.sin == std::sin(x[i]));
    }

}