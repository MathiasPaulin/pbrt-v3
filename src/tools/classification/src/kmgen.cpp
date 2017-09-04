#include "kmgen.h"
#include <glog/logging.h>
#include "core/geometry.h"
#include <memory>
#include <random>

// utility functions
float L2Norm(int vecsize, float *x, float *y) {
  float sum = 0.f;

  for (int i = 0; i < vecsize; ++i) {
    sum += (x[i]-y[i])*(x[i]-y[i]);
  }

  return std::sqrt(sum);
}

/*template <typename T>
T Label::geometricMean(const std::vector<pbrt::Point3f> &dataset) const {
  // TODO: overflow/precision error during mean/sqm computation ?
  constexpr int vecsz = 3; // vector dimension
  double mean[vecsz];

  for(int idx : elements) {
    pbrt::Point3f p = dataset[idx];
    for(int i = 0; i < vecsz; ++i) {
      mean[i] += p[i];
    }
  }

  for(int i = 0; i < vecsz; ++i) {
    mean[i] /= elements.size();
  }

  return pbrt::Point3f(mean[0], mean[1], mean[2]);
}
*/

namespace Kmeans {

void Classifier::run() {
  std::cerr << "Kmeans Classifier k = " << k << std::endl;

  // Generate random centroids
  for (int i = 0; i < k; ++i) {
    std::shared_ptr<Label> centroid = generator->generateRandomCentroid();
    labels.push_back(centroid);
  }

  while (!end()) {
    std::cerr << "Iteration " << iteration << std::endl;
    // Label each element
    for (int i = 0; i < paths.size(); ++i) {
      sortElement(i);
    }

    // Recompute new centroids
    recalculateCentroids();
    ++iteration;
  }
}

std::vector<std::shared_ptr<Label>> Classifier::getLabels() {
  // TODO: check state of classification before return
  return labels;
}

void Classifier::sortElement(int i) {
  int min_id = 0;
  float min_dist = labels[0]->distance(paths[i]);
  for (int j = 1; j < labels.size(); ++j) {
    if (labels[j]->distance(paths[i]) < min_dist) {
      min_dist = labels[j]->distance(paths[i]);
      min_id = j;
    }
  }

  labels[min_id]->label_element(paths[i], i);
}

void Classifier::recalculateCentroids() {
  for (int i = 0; i < labels.size(); ++i) {
    if(labels[i]->size() != 0) {
      labels[i]->recompute_centroid();
    } else {
      labels[i] = generator->generateRandomCentroid();
    }
  }
}

bool Classifier::end() {
  return maxiterations > 0 && iteration > maxiterations;

  /*
  // Else, check if centroids changed
  // TODO: distance threshold ?
  for(int i = 0; i < centroids.size(); ++i) {
    if(centroids[i] != oldcentroids[i]) {
      return false;
    }
  }

  return true; // Centroids converged
  */
}

DistanceGenerator::DistanceGenerator(const PathFile &p) : CentroidGenerator(p) {
  // Find bounds for the generator
  // TODO: collect useful data once from the file and give it to kmeans algortihm ?
  for (const pbrt::path_entry &path : paths) {
    pbrt::Vector3f od = pbrt::FromArray(path.vertices.back().v) - pbrt::FromArray(path.vertices[0].v);
    b = pbrt::Bounds3f(pbrt::Point3f(std::min(b.pMin.x, od.x), std::min(b.pMin.y, od.y), std::min(b.pMin.z, od.z)),
                       pbrt::Point3f(std::max(b.pMax.x, od.x), std::max(b.pMax.y, od.y), std::max(b.pMax.z, od.z)));
  }
}

std::shared_ptr<Label> DistanceGenerator::generateRandomCentroid() {
  // Generate a random vector between the probable vectors of the dataset
  // TODO: add some margin ?
  pbrt::Vector3f v;
  constexpr float margin = 0.f;
  for (int i = 0; i < 3; ++i) {
    v[i] = rng(generator) * (b.pMax[i]+margin - (b.pMin[i]-margin)) + (b.pMin[i]-margin);
  }

  return std::shared_ptr<Label>(new DistanceLabel(v));
}

float DistanceLabel::distance(const pbrt::path_entry &p) const {
  return std::fabs(pbrt::Vector3f(pbrt::FromArray(p.vertices.back().v) - pbrt::FromArray(p.vertices.front().v)).Length()
                   - length);
}

void DistanceLabel::recompute_centroid() {
  // Recompute length from average length of all vertices
  std::cerr << "Centroid update; label elements = "
            << elements.size() << ". Previous/current mean length = " << length << "/" << meanlength <<
                               " variance = " << sigma_sq << std::endl;

  length = meanlength;
  elements.clear();
}

void DistanceLabel::update_mean(const pbrt::path_entry &p) {
  const float length = pbrt::Vector3f(pbrt::FromArray(p.vertices.back().v) - pbrt::FromArray(p.vertices.front().v)).Length();
  if (!elements.empty()) {
    const float m_old = meanlength;
    meanlength += (length - meanlength) / elements.size();
    sigma_sq += (length-m_old)*(length-meanlength);
  }
  else {
    meanlength = length;
    sigma_sq = 0;
  }
}

#if 0
float PathDistance::distance(const pbrt::path_entry &p) {
  return distance(centroid, p);
}

void PathDistance::recompute_centroid() {
  // Recompute length from average length of all vertices
  std::cerr << "Centroid update; label elements = "
            << elements.size() << ". Previous/current mean length = " << length << "/" << meanlength <<
            " variance = " << sigma_sq << std::endl;

  length = meanlength;
  elements.clear();
}

float PathDistance::distance(const pbrt::path_entry &p1, const pbrt::path_entry &p2) {
  // find the closest match between paths
  const pbrt::path_entry &s_path = p1.pathlen < p2.pathlen ? p1 : p2;
  const pbrt::path_entry &l_path = p1.pathlen >= p2.pathlen ? p1 : p2;

  float distsum = 0.f;
  for (int i = 0; i < s_path.vertices.size(); ++i) {
    float localmin = std::numeric_limits<float>::max();
    for (int j = 0; j < l_path.vertices.size(); ++j) {
      localmin = std::min(pbrt::Vector3f(
              pbrt::FromArray(s_path.vertices[i].v) - pbrt::FromArray(l_path.vertices[j].v)).LengthSquared(), localmin);
    }
    distsum += localmin;
  }
  return std::sqrt(distsum); // geometric mean ?
}


void PathDistance::update_mean(const pbrt::path_entry &p) {
  if (!elements.empty()) {
    const float m_old = meanlength;
    meanlength += (last_distance - meanlength) / elements.size();
    sigma_sq += (last_distance-m_old)*(last_distance-meanlength);
  }
  else {
    meanlength = last_distance;
    sigma_sq = 0;
    currentcost = 0;
  }

  currentcost += last_distance;
}
#endif
} // namespace Kmeans