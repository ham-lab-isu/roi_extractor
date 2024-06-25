#include "boundary_order_generator/tsp_boundary_order_generator.h"

#include <algorithm>
#include <numeric>
#include <iostream>

namespace roi_detector
{
// Function to calculate distance matrix of boundary
auto CalculateDistanceMatrix(const Boundary boundary){
  // Compute distance matrix
  Eigen::MatrixXf distance_matrix(boundary.cols(), boundary.cols());

  for(Eigen::Index r = 0; r < distance_matrix.rows(); ++r)
  {
    for(Eigen::Index c = 0; c < distance_matrix.cols(); ++c)
    {
      distance_matrix(r, c) = (boundary.col(r) - boundary.col(c)).head<3>().squaredNorm();
    }
  }

  return distance_matrix;
}

// Function to calculate distance of current path through boundary for optimization
float CalculatePathDistance(std::vector<int> boundary_idxs, Eigen::MatrixXf distance_matrix)
{
  // path_distance starts at 0
  float path_distance = 0;

  // find distance from n point to n+1 point for every point, except for last point
  for(int i = 0; i < boundary_idxs.size()-1; i++){
    path_distance += distance_matrix(boundary_idxs[i],boundary_idxs[i+1]);
  }

  // add distance from final point to initial point
  path_distance += distance_matrix(boundary_idxs[boundary_idxs.size()-1],boundary_idxs[0]);

  return path_distance;
}

//Function to reorder points based on nearest neighbors
std::vector<int> NearestNeighborTSP(std::vector<int> boundary_idxs, Eigen::MatrixXf distance_matrix){

  // create new variable to hold the new path
  std::vector<int> new_boundary_idxs;
  new_boundary_idxs.reserve(boundary_idxs.size());

  // add initial point to new path
  new_boundary_idxs.push_back(boundary_idxs[0]);

  // remove initial point from old path
  boundary_idxs.erase(boundary_idxs.begin() + boundary_idxs[0]);

  int current_point = 0;
  int nearest_point;

  float current_distance;
  float best_distance;

  // select initial point
  while (boundary_idxs.size() != 0){

    // for each remaining point in the original path
    for(int boundary_idx : boundary_idxs){

      // find distance from current point to next point
      current_distance = distance_matrix(current_point, boundary_idx);

      // compare current candidate to best candidate
      // if current better than best or current is initial candidate, make new best
      if(current_distance < best_distance or boundary_idx == boundary_idxs[0]){
        best_distance = current_distance;
        nearest_point = boundary_idx; // storing nearest neighbor index
      }
    }

    // add point to path
    new_boundary_idxs.push_back(nearest_point);

    // move onto next point
    current_point = nearest_point;

    //remove current point from future consideration
    boundary_idxs.erase(std::find(boundary_idxs.begin(), boundary_idxs.end(), current_point));

  }

  boundary_idxs = new_boundary_idxs;

  return boundary_idxs;

}

std::vector<int> OptSwap(std::vector<int> boundary_idxs, int swap_first, int swap_last){

  while (swap_first < swap_last)
  {
    int temp = boundary_idxs[swap_first];
    boundary_idxs[swap_first] = boundary_idxs[swap_last];
    boundary_idxs[swap_last] = temp;
    swap_first++;
    swap_last--;
  }

  return boundary_idxs;
}

// 2 Opt Swap TSP Implementation
std::vector<int> TwoOptSwapTSP(std::vector<int> boundary_idxs, Eigen::MatrixXf distance_matrix){

  // create new variable to hold the new path
  std::vector<int> new_boundary_idxs;
  new_boundary_idxs.reserve(boundary_idxs.size());

  bool improved = true;

  while(improved){
    improved = false;

    for(int swap_first = 1; swap_first <= boundary_idxs.size() - 2; swap_first++){
      for(int swap_last = swap_first + 1; swap_last <= boundary_idxs.size()-1; swap_last++){
        new_boundary_idxs = OptSwap(boundary_idxs, swap_first, swap_last);
        if (CalculatePathDistance(new_boundary_idxs, distance_matrix) < CalculatePathDistance(boundary_idxs, distance_matrix)) {
          boundary_idxs = new_boundary_idxs;
          improved = true;
        } else{
          new_boundary_idxs = boundary_idxs;
        }
      }
    }
  }

  return boundary_idxs;
}

Boundary TSPBoundaryOrderGenerator::order(const Boundary& boundary) const
{

  // Creating vector of boundary indicies
  std::vector<int> boundary_idxs(boundary.cols());
  std::iota(boundary_idxs.begin(), boundary_idxs.end(), 0);

  // Calculate distance matrix for given path
  auto distance_matrix = CalculateDistanceMatrix(boundary);

  // Print initial distance
  std::cout << "Initial distance = " << CalculatePathDistance(boundary_idxs, distance_matrix) << std::endl;

  // Perform TSP
  boundary_idxs = NearestNeighborTSP(boundary_idxs, distance_matrix);

  // Print new distance
  std::cout << "Nearest Neighbor distance = " << CalculatePathDistance(boundary_idxs, distance_matrix) << std::endl;

  //Improve path from NN TSP with 2-Opt
  boundary_idxs = TwoOptSwapTSP(boundary_idxs, distance_matrix);

  // Print new distance
  std::cout << "TwoOptSwap distance = " << CalculatePathDistance(boundary_idxs, distance_matrix) << std::endl;

  // Reconstruct the ordered boundary
  Boundary output(4, boundary_idxs.size());
  for (Eigen::Index i = 0; i < boundary_idxs.size(); ++i)
    output.col(i) = boundary.col(boundary_idxs[i]);

  return output;
}

}

