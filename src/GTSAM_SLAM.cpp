#include <iostream>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "readG2OFile.h"

using namespace std;
using namespace gtsam;

int main(int argc, char** argv)
{
    NonlinearFactorGraph graph;
    Values initial;
    string filename = "../G2O_Files/input_INTEL_g2o.g2o";
    read2DG2O(filename, graph, initial);
    // initial.print("\nInitial Estimate:\n");
    return 0;
}
