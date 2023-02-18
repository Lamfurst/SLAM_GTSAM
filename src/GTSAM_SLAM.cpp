#include <iostream>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

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
                  Values& result, double tol = 1e-5, int maxIter = 1000);

// This function solves the 2D graph SLAM using incremental solution.
void solve2DIncremental(const NonlinearFactorGraph& graph, 
                        const Values& initialValue, 
                        Values& result);

int main(int argc, char** argv)
{
    // =========================================================================

    // The code for 1A is written in io.cpp and io.h

    // =========================================================================
    
    // The following code is for 1B 
    NonlinearFactorGraph graph;
    Values initialValue;
    Values result2DBatch;
    string inputfilename = "../G2O_Files/input_INTEL_g2o.g2o";

    // Read from G2O and build graph
    read2DG2O(inputfilename, graph, initialValue);

    
    string outputFilename;

    // Write initial values to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/initial2D.csv"; 
    exportValuesToCSV(outputFilename, initialValue, CSV_2D);

    // Solve using batch solution and export result to CSV file
    solve2DBatch(graph, initialValue, result2DBatch);

    // Write solution to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/result2DBatch.csv";
    exportValuesToCSV(outputFilename, result2DBatch, CSV_2D);

    // =========================================================================
    
    // The following code is for 1C
    Values result2DIncremental;

    // Solve using batch solution and export result to CSV file
    solve2DIncremental(graph, initialValue, result2DIncremental);

    // Write solution to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/result2DIncremental.csv";
    exportValuesToCSV(outputFilename, result2DIncremental, CSV_2D);

    // =========================================================================

    
    return 0;
}

void solve2DBatch(NonlinearFactorGraph& graph, Values& initialValue,
                  Values& result, double tol, int maxIter)
{
    GaussNewtonParams parameters;
    parameters.relativeErrorTol = tol;
    parameters.maxIterations = maxIter;

    GaussNewtonOptimizer optimizer(graph, initialValue, parameters);
    result = optimizer.optimize();
}

void solve2DIncremental(const NonlinearFactorGraph& graph, 
                        const Values& initialValue, 
                        Values& result)
{
    ISAM2 isam;
    const size_t poseNum = initialValue.size();
    for (size_t i = 0; i < poseNum; ++i)
    {
        NonlinearFactorGraph iGraph;
        Values iValues;
        Pose2 pose = initialValue.at(i).cast<Pose2>();
        if (i == 0)
        {
            auto priorNoise = noiseModel::Diagonal::
                              Sigmas(Vector3(0.3, 0.3, 0.1));
            iGraph.add(PriorFactor<Pose2>(i, pose, priorNoise));
            iValues.insert(i, pose);
        }
        else
        {
            Pose2 prevPose = result.at(i - 1).cast<Pose2>();
            iValues.insert(i, prevPose);

            size_t numEdges = graph.size();
            for (size_t j = 1; j < numEdges; ++j)
            {
                const auto edge = graph.at(j);
                const auto keys = edge->keys();
                if (keys[1] == i)
                {
                    iGraph.add(edge);
                }
            }
        }
        isam.update(iGraph, iValues);
        result = isam.calculateEstimate();
    }
}
