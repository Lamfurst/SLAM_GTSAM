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

#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

void readG2O(const string& filename, 
             NonlinearFactorGraph& graph,
             Values& initialValue,
             const bool is2D)
{
    ifstream file(filename);
    string line;

    // Add prior factor to graph
    if (is2D)
    {
        Pose2 priorMean(0.0, 0.0, 0.0);
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
        graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));
    }
    else
    {
        Pose3 priorMean(Quaternion(0.0, 0.0, 0.0, 0.0), Point3(0.0, 0.0, 1.0));
        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector6(0.3, 0.3, 0.3,
                                                               0.1, 0.1, 0.1));
        graph.add(PriorFactor<Pose3>(1, priorMean, priorNoise));
    }
    

    // Read from G2O file
    while (getline(file, line))
    {
        istringstream iss(line);
        string tag;
        iss >> tag;

        // Read 2D G2O Files
        if (is2D)
        {
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

                Matrix3 infoMatrix;
                buildInfoMatrix<Matrix3>(3, iss, infoMatrix);
                
                Matrix3 covMatrix = infoMatrix.inverse();
                auto odomNoise = noiseModel::Gaussian::Covariance(covMatrix);

                graph.add(BetweenFactor<Pose2>(firstKey, secondKey, pose, 
                                               odomNoise));
            }
        }

        // Read 3D G2O Files
        else
        {
            Key i;
            double x, y, z, qx, qy, qz, qw;
            // Read the Values
            if (tag == "VERTEX_SE3:QUAT")
            {
                Key key;
                double x, y, z, qx, qy, qz, qw;
                iss >> key >> x >> y >> z >> qx >> qy >> qz >> qw;
                Pose3 pose(Quaternion(qw, qx, qy, qz), Point3(x, y, z));
                initialValue.insert(key, pose);
            }

            // Read the Factors
            else if (tag == "EDGE_SE3:QUAT")
            {
                Key firstKey, secondKey;
                double x, y, z, qx, qy, qz, qw;

                iss >> firstKey >> secondKey;
                iss >> x >> y >> z >> qx >> qy >> qz >> qw;
                Pose3 pose(Quaternion(qw, qx, qy, qz), Point3(x, y, z));
                                
                Matrix6 infoMatrix;
                buildInfoMatrix<Matrix6>(6, iss, infoMatrix);

                Matrix6 covMatrix = infoMatrix.inverse();
                auto odomNoise = noiseModel::Gaussian::Covariance(covMatrix);

                graph.add(BetweenFactor<Pose3>(firstKey, secondKey, pose, 
                                               odomNoise));

            }
        }
        
    }
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
        outfile << "x,y,z" << endl;
        for (const auto& key_value : initialValue)
        {
            const auto pose = key_value.value.cast<Pose3>();
            outfile << pose.x() << "," << pose.y() << "," << pose.z() << endl;
        }
    }

    outfile.close();
}

template <typename M>
void buildInfoMatrix(size_t size, istringstream& iss, M& infoMatrix)
{
    for (int i = 0; i < size; ++i)
    {
        for (int j = i; j < size; ++j)
        {
            double curQ;
            iss >> curQ;
            infoMatrix(i, j) = curQ;
        }
    }

    infoMatrix = infoMatrix.template selfadjointView<Eigen::Upper>();
}