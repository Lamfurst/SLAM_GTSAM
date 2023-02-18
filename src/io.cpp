#include "io.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Matrix.h>

using namespace std;
using namespace gtsam;

void read2DG2O(const string& filename, 
               NonlinearFactorGraph& graph,
               Values& initialValue)
{
    ifstream file(filename);
    string line;

    // Add prior factor to graph
    Pose2 priorMean(0.0, 0.0, 0.0);
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

    // Read from G2O file
    while (getline(file, line))
    {
        istringstream iss(line);
        string tag;
        iss >> tag;

        // Read the Values
        if (tag == "VERTEX_SE2")
        {
            Key key;
            double x, y, theta;

            iss >> key >> x >> y >> theta;
            Pose2 pose(x, y, theta);
            initialValue.insert(key, pose);
        }

        // Read Factors
        else if (tag == "EDGE_SE2")
        {
            Key firstKey, secondKey;
            double x, y, theta;
            double q11, q12, q13, q22, q23, q33;

            iss >> firstKey >> secondKey;

            iss >> x >> y >> theta;
            Pose2 pose(x, y, theta);

            iss >> q11 >> q12 >> q13 >> q22 >> q23 >> q33;
            Matrix3 infoMatrix;
            infoMatrix(0, 0) = q11;
            infoMatrix(0, 1) = q12;
            infoMatrix(0, 2) = q13;
            infoMatrix(1, 0) = q12;
            infoMatrix(1, 1) = q22;
            infoMatrix(1, 2) = q23;
            infoMatrix(2, 0) = q13;
            infoMatrix(2, 1) = q23;
            infoMatrix(2, 2) = q33;
            
            Matrix3 covMatrix = infoMatrix.inverse();
            auto odometryNoise = noiseModel::Gaussian::Covariance(covMatrix);
            graph.add(BetweenFactor<Pose2>(firstKey, secondKey, 
                                           pose, odometryNoise));

        }
    }
}

void read3DG2O(const string& filename,
               NonlinearFactorGraph& graph,
               Values& initialValue)
{

}

void exportValuesToCSV(const string& filename, Values& initialValue, 
                       const bool is2D)
{
    ofstream outfile(filename);
    // If this is 2D value
    if (is2D)
    {
        outfile << "x,y" << endl;
        for (const auto& key_value : initialValue)
        {
            const auto pose = key_value.value.cast<Pose2>();
            outfile << pose.x() << "," << pose.y() << endl;
        }
    }
    else
    {
        ;
    }

    outfile.close();
}