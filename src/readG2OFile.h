#ifndef _READG2OFILE_H_
#define _READG2OFILE_H_

#include <string>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

// Should I include <iostream> here?
// Where should I use using namespace?
// is it a good practice to use using namespace?

// Use this function to read 2D G2O File
void read2DG2O(const std::string& filename, 
               gtsam::NonlinearFactorGraph& graph,
               gtsam::Values& initialValue);

// Use this function to read 3D G2O File
void read3DG2O(const std::string& filename, 
               gtsam::NonlinearFactorGraph& graph,
               gtsam::Values& initialValue);
#endif