#include <iostream>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "io.h"

using namespace std;
using namespace gtsam;

// Programmer: Junkai Zhang, February 16th 2023

// Purpose: This program builds NonlinearFactorGraph from G2O files. The graph
// is then optimized using batch method and increamental method. The output of
// the optimized solution is output with csv file, which is used to plot in
// Matlab

const bool CSV_2D = true;
const bool CSV_3D = false;

// This function solves the 2D graph SLAM using batch solution.
void solve2DBatch(NonlinearFactorGraph& graph, Values& initialValue, 
                  Values& result, double tol, int maxIter);

// This function solves the 2D graph SLAM using incremental solution.
void solve2DIncremental ();

int main(int argc, char** argv)
{
    // =========================================================================

    // The code for 1A is written in io.cpp and io.h

    // =========================================================================
    
    // The following code is for 1B 
    NonlinearFactorGraph graph;
    Values initialValue;
    Values result;
    string inputfilename = "../G2O_Files/input_INTEL_g2o.g2o";

    // Read from G2O and build graph
    read2DG2O(inputfilename, graph, initialValue);

    // Write initial values to csv for plotting
    string outputFilename;
    outputFilename = "../CSV/initial.csv";
    exportValuesToCSV(outputFilename, initialValue, CSV_2D);

    // Solve using batch solution and export result to CSV file
    double tol = 1e-5;
    int maxIter = 1000;
    solve2DBatch(graph, initialValue, result, tol, maxIter);
    // Write solution to csv for plotting
    outputFilename = "../CSV/resultBatch.csv";
    exportValuesToCSV(outputFilename, result, CSV_2D);

    // =========================================================================

    
    
    return 0;
}

void solve2DBatch(NonlinearFactorGraph& graph, Values& initialValue,
                  Values& result, double tol, int maxIter)
{
    // Add prior factor to graph
    Pose2 priorMean(0.0, 0.0, 0.0);
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

    GaussNewtonParams parameters;
    parameters.relativeErrorTol = tol;
    parameters.maxIterations = maxIter;

    GaussNewtonOptimizer optimizer(graph, initialValue, parameters);
    result = optimizer.optimize();
    // result = LevenbergMarquardtOptimizer(graph, initialValue).optimize();
}

