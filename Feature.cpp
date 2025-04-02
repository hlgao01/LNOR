#include "Feature.h"

// Initializes a member variable
std::vector<std::pair<int, Eigen::Vector3f>> Feature::pointFeature;
Feature::Feature(PointCloud::Ptr cloud, int N)
    : inputCloud(cloud),num(N) {
    outputCloud.reset(new PointCloudRGB);
    kdtree.reset(new pcl::search::KdTree<Point>());
}

void Feature::calculateFeatures() {
    kdtree->setInputCloud(inputCloud);
    pointFeature.reserve(inputCloud->points.size());
    //// sequence
    //for (int i = 0; i < num; ++i) {
    //    float k = 10 + 90 * std::log(1 + i) / std::log(num);
    //    sequence.push_back(static_cast<int>(std::round(k)));
    //}
    //int i = 0;
    for (const auto& searchPoint : inputCloud->points) {
        std::pair<int, Eigen::Vector3f> bestFeature;
        computeFeatureForPoint(searchPoint, bestFeature);
        //i++;
        //std::cout << i << "/" << inputCloud->size() << std::endl;
        // Store the characteristics of the computation
        pointFeature.push_back(bestFeature);
    }
}

void Feature::computeFeatureForPoint(const Point& searchPoint, std::pair<int, Eigen::Vector3f>& bestFeature) const {
    float minEntropy = std::numeric_limits<float>::max();
    int bestShapeIndex = -1; // 1: linear, 2: surface, 3: scatter
    Eigen::Vector3f bestNormal;

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    kdtree->nearestKSearch(searchPoint, 20, pointIdxNKNSearch, pointNKNSquaredDistance);
    Eigen::Matrix3f covarianceMatrix = Eigen::Matrix3f::Zero();
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*inputCloud, pointIdxNKNSearch, xyz_centroid);

    for (const int idx : pointIdxNKNSearch) {
        Eigen::Vector4f pt(inputCloud->points[idx].x, inputCloud->points[idx].y, inputCloud->points[idx].z, 1);
        pt -= xyz_centroid;
        covarianceMatrix += pt.head<3>() * pt.head<3>().transpose();
    }

    covarianceMatrix /= static_cast<float>(pointIdxNKNSearch.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covarianceMatrix);
    Eigen::Vector3f eigenvalues = eigensolver.eigenvalues();

    float linear = (eigenvalues(2) - eigenvalues(1)) / eigenvalues(2);
    float surface = (eigenvalues(1) - eigenvalues(0)) / eigenvalues(2);
    float scatter = eigenvalues(0) / eigenvalues(2);

    Eigen::Vector3f feature(linear, surface, scatter);
    bestShapeIndex = std::max_element(feature.data(), feature.data() + 3) - feature.data() + 1;
    bestNormal = eigensolver.eigenvectors().col(0); // Normal vector

    //// Use this code if using multiple k neighborhoods to determine the best feature
    //for (int k : sequence)
    //{
    //    std::vector<int> pointIdxNKNSearch;
    //    std::vector<float> pointNKNSquaredDistance;
    //    kdtree->nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);
    //    Eigen::Matrix3f covarianceMatrix = Eigen::Matrix3f::Zero();
    //    Eigen::Vector4f xyz_centroid;
    //    pcl::compute3DCentroid(*inputCloud, pointIdxNKNSearch, xyz_centroid);

    //    for (const int idx : pointIdxNKNSearch) {
    //        Eigen::Vector4f pt(inputCloud->points[idx].x, inputCloud->points[idx].y, inputCloud->points[idx].z, 1);
    //        pt -= xyz_centroid;
    //        covarianceMatrix += pt.head<3>() * pt.head<3>().transpose();
    //    }

    //    covarianceMatrix /= static_cast<float>(pointIdxNKNSearch.size());
    //    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covarianceMatrix);
    //    Eigen::Vector3f eigenvalues = eigensolver.eigenvalues();

    //    float linear = (eigenvalues(2) - eigenvalues(1)) / eigenvalues(2);
    //    float surface = (eigenvalues(1) - eigenvalues(0)) / eigenvalues(2);
    //    float scatter = eigenvalues(0) / eigenvalues(2);

    //    Eigen::Vector3f feature(linear, surface, scatter);
    //    float entropy = -linear * log(linear) - surface * log(surface) - scatter * log(scatter);

    //    if (entropy < minEntropy) {
    //        minEntropy = entropy;
    //        bestShapeIndex = std::max_element(feature.data(), feature.data() + 3) - feature.data() + 1;
    //        bestNormal = eigensolver.eigenvectors().col(0); // Normal vector
    //    }

    //}
    bestFeature = { bestShapeIndex, bestNormal };
}

Feature::PointCloudRGB::Ptr Feature::getOutputCloud() {
    for (size_t i = 0; i < inputCloud->points.size(); ++i) {
        const auto& searchPoint = inputCloud->points[i];
        const auto& feature = pointFeature[i];

        pcl::PointXYZRGB colorPoint;
        colorPoint.x = searchPoint.x;
        colorPoint.y = searchPoint.y;
        colorPoint.z = searchPoint.z;

        switch (feature.first) {
        case 1: // linear
            colorPoint.r = 255;
            colorPoint.g = 0;
            colorPoint.b = 0;
            break;
        case 2: // surface
            colorPoint.r = 0;
            colorPoint.g = 0;
            colorPoint.b = 255;
            break;
        case 3: // scatter
            colorPoint.r = 255;
            colorPoint.g = 255;
            colorPoint.b = 0;
            break;
        }

        outputCloud->points.push_back(colorPoint);
    }

    outputCloud->width = outputCloud->points.size();
    outputCloud->height = 1;
    outputCloud->is_dense = true;

    return outputCloud;
}
