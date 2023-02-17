#ifndef _IO_H_
#define _IO_H_

#include <string>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// Programmer: Junkai Zhang, February 16th 2023

// Purpose: This is a simple file helps to read G2O files and convert the
// G2O file to the GTSAM graph file needed. Also this file helps to output the
// Values in GTSAM to csv file need for plotting in Matlab.


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

// I don't know why my matplotcpp does not work. So I use this function to
// export csv file to plot in Matlab
void exportValuesToCSV(const std::string& filename, gtsam::Values& initialValue, 
                       const bool is2D);


#endif