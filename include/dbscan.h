#pragma once

#include <vector>

namespace dbscan
{
	double EuclideanDistance(const std::vector<double>& A, const std::vector<double>& B);
	std::vector<int>* DBScan(const std::vector<std::vector<double>>& data, double epsilon, int minPts, int& numGroups);
}
