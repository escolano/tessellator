#pragma once

#include <queue>
#include <map>
#include <utility>

#include "Mesh.h"
#include "Progress.h"

namespace meshlib {

struct Options {
    ProgressManager progress;
    int decimalPlacesInCollapser;
    Options() {
      decimalPlacesInCollapser=4;
    }
};

}

