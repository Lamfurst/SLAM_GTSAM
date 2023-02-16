#include "readG2OFile.h"

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
// Programmer: Junkai Zhang, February 16th 2023

// Purpose: This is a simple file helps to read G2O files and convert the
// G2O file to the GTSAM graph file needed.

void read2DG2O(const string& filename, 
               NonlinearFactorGraph& graph,
               Values& initialValue)
{
    ifstream file(filename);
    string line;

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

            auto odometryNoise = noiseModel::Gaussian::Information(infoMatrix);
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