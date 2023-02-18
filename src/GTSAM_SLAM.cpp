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

// =============================================================================

const bool CSV_2D = true;
const bool CSV_3D = false;

// This function solves graph SLAM using batch solution.
void solveBatch(NonlinearFactorGraph& graph, Values& initialValue, 
                Values& result, double tol = 1e-5, int maxIter = 1000);

// This template function solves the graph SLAM using incremental solution.
template <typename Pose>
void solveIncremental(const NonlinearFactorGraph& graph, 
                      const Values& initialValue, Values& result,
                      const bool is2D);

// =============================================================================

int main(int argc, char** argv)
{
    // =========================================================================

    // The code for 1A is written in io.cpp and io.h

    // =========================================================================
    
    // The following code is for 1B 
    NonlinearFactorGraph graph;
    Values initialValue;

    // Read from G2O and build graph
    // TODO: Change the G2O file location
    string inputfilename = "../G2O_Files/input_INTEL_g2o.g2o";
    readG2O(inputfilename, graph, initialValue, CSV_2D);

    string outputFilename;
    // Write initial values to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/initial2D.csv"; 
    exportValuesToCSV(outputFilename, initialValue, CSV_2D);

    // Solve using batch solution and export result to CSV file
    Values result2DBatch;
    solveBatch(graph, initialValue, result2DBatch);

    // Write solution to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/result2DBatch.csv";
    exportValuesToCSV(outputFilename, result2DBatch, CSV_2D);

    // =========================================================================
    
    // The following code is for 1C
    Values result2DIncremental;

    // Solve using batch solution and export result to CSV file
    // solve2DIncremental(graph, initialValue, result2DIncremental);
    solveIncremental<Pose2>(graph, initialValue, result2DIncremental,CSV_2D);

    // Write solution to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/result2DIncremental.csv";
    exportValuesToCSV(outputFilename, result2DIncremental, CSV_2D);

    // =========================================================================

    // The following code is for Q2
    NonlinearFactorGraph graph3D;
    Values initialValue3D;

    // Read from G2O and build graph
    // TODO: Change the G2O file location
    inputfilename = "../G2O_Files/parking-garage.g2o";
    readG2O(inputfilename, graph3D, initialValue3D, CSV_3D);

    // Write initial values to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/initial3D.csv"; 
    exportValuesToCSV(outputFilename, initialValue3D, CSV_3D);

    // =========================================================================

    // The following code is for 2B

    Values result3DBatch;
    // // Solve using batch solution and export result to CSV file
    solveBatch(graph3D, initialValue3D, result3DBatch);

    // Write solution to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/result3DBatch.csv";
    exportValuesToCSV(outputFilename, result3DBatch, CSV_3D);

    // =========================================================================

    // The following code is for 2C

    Values result3DIncremental;

    // Solve using batch solution and export result to CSV file
    // solve2DIncremental(graph, initialValue, result2DIncremental);
    solveIncremental<Pose3>(graph3D, initialValue3D, 
                            result3DIncremental, CSV_3D);

    // Write solution to csv for plotting
    // TODO: Change the outputFilename to your folder
    outputFilename = "../CSV/result3DIncremental.csv";
    exportValuesToCSV(outputFilename, result3DIncremental, CSV_3D);
    
    return 0;
}

// =============================================================================

void solveBatch(NonlinearFactorGraph& graph, Values& initialValue,
                Values& result, double tol, int maxIter)
{
    GaussNewtonParams parameters;
    parameters.relativeErrorTol = tol;
    parameters.maxIterations = maxIter;

    GaussNewtonOptimizer optimizer(graph, initialValue, parameters);
    result = optimizer.optimize();
}

// =============================================================================

template <typename Pose>
void solveIncremental(const NonlinearFactorGraph& graph, 
                      const Values& initialValue, Values& result, 
                      const bool is2D)
{
    ISAM2 isam;
    const size_t poseNum = initialValue.size();
    for (size_t i = 0; i < poseNum; ++i)
    {
        NonlinearFactorGraph iGraph;
        Values iValues;
        Pose pose = initialValue.at(i).cast<Pose>();
        if (i == 0)
        {
            if (is2D)
            {
                auto priorNoise = noiseModel::Diagonal::
                                  Sigmas(Vector3(0.3, 0.3, 0.1));
                iGraph.add(PriorFactor<Pose>(i, pose, priorNoise));
            }
            else
            {
                auto priorNoise = noiseModel::Diagonal::
                                  Sigmas(Vector6(0.3, 0.3, 0.3, 0.1, 0.1, 0.1));
                iGraph.add(PriorFactor<Pose>(i, pose, priorNoise));  
            }
            iValues.insert(i, pose);
        }
        else
        {
            Pose prevPose = result.at(i - 1).cast<Pose>();
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

// =============================================================================