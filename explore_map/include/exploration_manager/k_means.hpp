#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <numeric>

namespace ExploreMap
{
class KMeans {
public:
    KMeans(int k) : k(k) {}

    void fit(const std::vector<geometry_msgs::Point>& data) {
        if (data.empty()) {
            std::cerr << "Veri seti boş." << std::endl;
            return;
        }

        centroids = initializeCentroids(data);

        for (int iter = 0; iter < maxIterations; ++iter) {
            assignToClusters(data);

            std::vector<geometry_msgs::Point> newCentroids = calculateCentroids(data);

            if (centroids == newCentroids) {
                std::cout << "Iterasyon sayısı: " << iter + 1 << std::endl;
                break;
            }

            centroids = newCentroids;
        }
    }

    const std::vector<int>& getLabels() const {
        return labels;
    }

    std::vector<geometry_msgs::Point>& getCentroids() {
        return centroids;
    }

private:
    int k;
    std::vector<int> labels;
    std::vector<geometry_msgs::Point> centroids;

    const int maxIterations = 100;

    std::vector<geometry_msgs::Point> initializeCentroids(const std::vector<geometry_msgs::Point>& data) {
        srand(time(0));
        std::vector<geometry_msgs::Point> initialCentroids;
        std::vector<int> indices(data.size());

        std::iota(indices.begin(), indices.end(), 0);
        std::random_shuffle(indices.begin(), indices.end());

        for (int i = 0; i < k; ++i) {
            initialCentroids.push_back(data[indices[i]]);
        }

        return initialCentroids;
    }

    void assignToClusters(const std::vector<geometry_msgs::Point>& data) {
        labels.clear();

        for (const auto& point : data) {
            int closestCentroid = findClosestCentroid(point);
            labels.push_back(closestCentroid);
        }
    }

    int findClosestCentroid(const geometry_msgs::Point& point) {
        int closestCentroid = 0;
        double minDistance = std::numeric_limits<double>::max();

        for (int i = 0; i < k; ++i) {
            double distance = calculateDistance(point, centroids[i]);

            if (distance < minDistance) {
                minDistance = distance;
                closestCentroid = i;
            }
        }

        return closestCentroid;
    }

    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    std::vector<geometry_msgs::Point> calculateCentroids(const std::vector<geometry_msgs::Point>& data) {
        std::vector<int> clusterCounts(k, 0);
        std::vector<geometry_msgs::Point> newCentroids(k, geometry_msgs::Point());

        for (size_t i = 0; i < data.size(); ++i) {
            int clusterIndex = labels[i];
            clusterCounts[clusterIndex]++;

            newCentroids[clusterIndex].x += data[i].x;
            newCentroids[clusterIndex].y += data[i].y;
            newCentroids[clusterIndex].z += data[i].z;
        }

        for (int i = 0; i < k; ++i) {
            if (clusterCounts[i] > 0) {
                newCentroids[i].x /= clusterCounts[i];
                newCentroids[i].y /= clusterCounts[i];
                newCentroids[i].z /= clusterCounts[i];
            }
        }

        return newCentroids;
    }
};


}
