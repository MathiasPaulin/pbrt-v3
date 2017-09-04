#include "kmedoids.h"
#include <glog/logging.h>
#include "core/geometry.h"
#include <memory>
#include <atomic>
#include "pbrt.h"
#include "core/parallel.h"
#include <random>

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

namespace Kmedoids {

void Classifier::run() {
  std::cerr << "Kmedoids Classifier k = " << k << std::endl;

  // Select only a few paths
  std::random_device rd;
  std::mt19937 g(rd());
  std::uniform_int_distribution<unsigned long> dist(0, paths.size());

  std::set<uint64_t> samples;
  while (samples.size() < samplesize) {
    samples.insert(dist(g));
  }

  sampleset.resize(samplesize);
  sampleset.assign(samples.begin(), samples.end());


  // Generate random centroids
  for (int i = 0; i < k; ++i) {
    std::shared_ptr<Label> centroid; //= generator->generateRandomCentroid();
    bool redundant = true;
    while(redundant) {
      centroid = generator->generateRandomCentroid();
      for(std::shared_ptr<Label> label : labels) {
        if( *centroid == *label )
          continue;
      }
      redundant = false;
    }
    labels.push_back(centroid);
  }

  cost = std::numeric_limits<size_t>::max();
  int stablemedoids = 0;
  while (!end() && stablemedoids != labels.size()) {
    lastcost = cost; // update cost.
    std::cerr << std::endl << "Iteration " << iteration << std::endl;
    /*
    // Label each element
    for (int i = 0; i < paths.size(); ++i) {
      sortElement(i);
    }
    */
    while(!sampleset.empty()) {
      sortElement(sampleset[0]);
      sampleset.erase(sampleset.begin());
    }

    // Sum up all label costs
    cost = 0;
    int totalpaths = 0;



    // Recompute new centroids
    recalculateCentroids();
    // Recover centroids that need to be resorted
    stablemedoids = 0;
    for(std::shared_ptr<Label> label : labels) {
      stablemedoids += (label->fixed_medoid() ? 1 : 0);
	    label->getElementsToSort(sampleset);
      cost += label->cost();
      totalpaths += label->size();
    }
    std::cerr << "n = " << totalpaths << " (+" << sampleset.size() << ") ; Total cost for iteration " << iteration << " : " << cost << " (" << cost-lastcost << ")" << std::endl;

    ++iteration;
  }

  std::cerr << "Assign remaining paths" << std::endl;
  for(int64_t i = 0; i < paths.size(); ++i) {
    if(samples.find(i) == samples.end()) {
      sortElement(i);
    }
  }
}

std::vector<std::shared_ptr<Label>> Classifier::getLabels() {
  // TODO: check state of classification before return
  return labels;
}

void Classifier::sortElement(int i) {
  int min_id = 0;
  float min_dist = labels[0]->distance(paths[i]);
  //pbrt::ParallelFor([&](uint64_t j) {
  for (int j = 1; j < labels.size(); ++j) {
    if (labels[j]->distance(paths[i]) < min_dist) {
      min_dist = labels[j]->distance(paths[i]);
      min_id = j;
    }
  } //, labels.size());

  labels[min_id]->label_element(paths[i], i);
}

void Classifier::recalculateCentroids() {
  for (int i = 0; i < labels.size(); ++i) {
    if(labels[i]->size() != 0) {
      labels[i]->recompute_centroid(paths);
    } else {
      labels[i] = generator->generateRandomCentroid();
    }

    std::shared_ptr<Label> centroid = labels[i];
    bool redundant = true;
    while(redundant) {
      redundant = false;
      for(int j = 0; j < labels.size(); ++j) {
        if( i != j && *centroid == *labels[j]) {
          labels[j]->elements.insert(labels[j]->elements.begin(), centroid->elements.begin(), centroid->elements.end());
          centroid = generator->generateRandomCentroid();
	        redundant = true;
	      }
      }
    }

    labels[i] = centroid;
  }

}

bool Classifier::end() {
  // Stop if global cost increases
  return (maxiterations > 0 && iteration > maxiterations); // || (iteration > 0 && cost > lastcost);

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

// ray origin -> destination


DistanceGenerator::DistanceGenerator(const PathFile &p) : CentroidGenerator(p) {
  // Find bounds for the generator
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

float DistanceLabel::distance(const pbrt::path_entry &p) {
  return distance(p, length);
}

void DistanceLabel::recompute_centroid(const PathFile &pathfile) {
  std::cerr << "Centroid recomputation";
  std::atomic<float> min_dist;
  std::atomic<uint64_t> best_candidate;


  min_dist.store(std::numeric_limits<float>::max());
  best_candidate.store(0);

  //pbrt::ParallelInit();
  pbrt::ParallelFor([&](uint64_t i) {
      pbrt::path_entry p = pathfile[elements[i]];
      float locallength = std::fabs(pbrt::Vector3f(pbrt::FromArray(p.vertices.back().v) - pbrt::FromArray(p.vertices.front().v)).Length());
      float localdistsum = distance(p); // Account for distance to current mean
      for (int j = 0; j < elements.size(); ++j) {
        localdistsum += distance(pathfile[elements[j]], locallength);
        
        /*
        if (localdistsum > min_dist)
          break;
        */
      }

      if (localdistsum < min_dist) {
        min_dist = localdistsum;
        best_candidate = i;
      }

  }, elements.size());

  if (min_dist < currentcost) {
    // Update medoid
    centroid = pbrt::Vector3f(pbrt::FromArray(pathfile[elements[best_candidate]].vertices.back().v) - pbrt::FromArray(pathfile[elements[best_candidate]].vertices.front().v));
    length = centroid.Length();
    // centroid = pathfile[best_candidate].path;
    std::cerr << "Centroid update; ";
    resortelements = true;
  } else {
    std::cerr << "Keep medoid; ";
  }

  std::cerr << "label " << centroid << " ; length = " << length << " elements = "
            << elements.size() << ". Previous/current cost = " << currentcost << "/" << min_dist <<
            " variance = " << sigma_sq << std::endl;
}

void DistanceLabel::update_mean(const pbrt::path_entry &p) {
  const float length = pbrt::Vector3f(pbrt::FromArray(p.vertices.back().v) - pbrt::FromArray(p.vertices.front().v)).Length();
  if (!elements.empty()) {
    const float m_old = meanlength;
    meanlength += (length - meanlength) / elements.size();
    sigma_sq += (length-m_old)*(length-meanlength);
    currentcost += length;
  }
  else {
    meanlength = length;
    sigma_sq = 0;
    currentcost = 0;
  }
}

float DistanceLabel::distance(const pbrt::path_entry &p, float centroidlength) const {
  return std::fabs(pbrt::Vector3f(pbrt::FromArray(p.vertices.back().v) - pbrt::FromArray(p.vertices.front().v)).Length()
                   - centroidlength);
}

void DistanceLabel::getElementsToSort(std::vector<uint64_t> &sampleset) {
  if(!resortelements)
    return;

  sampleset.insert(sampleset.end(), elements.begin(), elements.end());
  elements.clear();
  resortelements = false;
}

// Levenshtein Distance

std::shared_ptr<Label> LevenshteinGenerator::generateRandomCentroid() {
  // Generate a random vector between the probable vectors of the dataset

  uint64_t random_id(rng(generator) * paths.size());

  return std::shared_ptr<Label>(new LevenshteinDistance(paths[random_id].path));
}

int LevenshteinDistance::distance(const std::string &s1, const std::string &s2) const {
	// To change the type this function manipulates and returns, change
	// the return type and the types of the two variables below.
	int s1len = s1.size();
	int s2len = s2.size();

	auto column_start = (decltype(s1len))1;

	auto column = new decltype(s1len)[s1len + 1];
	std::iota(column + column_start, column + s1len + 1, column_start);

	for (auto x = column_start; x <= s2len; x++) {
		column[0] = x;
		auto last_diagonal = x - column_start;
		for (auto y = column_start; y <= s1len; y++) {
			auto old_diagonal = column[y];
			auto possibilities = {
				column[y] + 1,
				column[y - 1] + 1,
				last_diagonal + (s1[y - 1] == s2[x - 1]? 0 : 1)
			};
			column[y] = std::min(possibilities);
			last_diagonal = old_diagonal;
		}
	}
	auto result = column[s1len];
	delete[] column;
	return result;
}

float LevenshteinDistance::distance(const pbrt::path_entry &p) {
  return last_distance = distance(centroid, p.path);
}

void LevenshteinDistance::recompute_centroid(const PathFile &p) {
  std::cerr << "Centroid recomputation";
  std::atomic<uint64_t> min_dist;
  min_dist.store(std::numeric_limits<unsigned long>::max());
  std::atomic<uint64_t> best_candidate;
  best_candidate.store(0);

  //pbrt::ParallelInit();
  pbrt::ParallelFor([&](uint64_t i) {
    std::string s(p[elements[i]].path);
    uint64_t localdistsum = distance(s, centroid); // Account for distance to current mean
    for (int j = 0; j < elements.size(); ++j) {
      localdistsum += distance(s, p[elements[j]].path);
      /*
      if (localdistsum > min_dist)
        break;
      */
    }

    if (localdistsum < min_dist) {
      min_dist = localdistsum;
      best_candidate = i;
    }

  }, elements.size());

  if(min_dist < currentcost) {
    // Update medoid
    centroid = p[elements[best_candidate]].path;
    std::cerr << "Centroid update; ";
    resortelements = true;
  } else {
    std::cerr << "Keep medoid; ";
  }

  std::cerr <<  "label " << centroid << " elements = "
            << elements.size() << ". Previous/current cost = " << currentcost<< "/" << min_dist <<
            " variance = " << sigma_sq << std::endl;
}

void LevenshteinDistance::getElementsToSort(std::vector<uint64_t> &sampleset) {
  if(!resortelements)
    return;

  sampleset.insert(sampleset.end(), elements.begin(), elements.end());
  elements.clear();
  resortelements = false;
}

void LevenshteinDistance::update_mean(const pbrt::path_entry &p) {
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

float PathDistance::distance(const pbrt::path_entry &p) {
  return last_distance = distance(centroid, p);
}

// todo: generic centroid recomputation
void PathDistance::recompute_centroid(const PathFile &p) {
  std::cerr << "Centroid recomputation";
  std::atomic<uint64_t> min_dist;
  min_dist.store(std::numeric_limits<unsigned long>::max());
  std::atomic<uint64_t> best_candidate;
  best_candidate.store(0);

  pbrt::ParallelFor([&](uint64_t i) {
      float localdistsum = distance(p[elements[i]]); // Account for distance to current mean
      for (int j = 0; j < elements.size(); ++j) {
        localdistsum += distance(p[elements[i]], p[elements[j]]);
      }

      if (localdistsum < min_dist) {
        min_dist = localdistsum;
        best_candidate = i;
      }

  }, elements.size());

  if(min_dist < currentcost) {
    // Update medoid
    centroid = p[elements[best_candidate]];
    std::cerr << "Centroid update; ";
    resortelements = true;
  } else {
    std::cerr << "Keep medoid; ";
  }

  std::cerr <<  "label " << centroid.path << " elements = "
            << elements.size() << ". Previous/current cost = " << currentcost<< "/" << min_dist <<
            " variance = " << sigma_sq << std::endl;
}

float PathDistance::distance(const pbrt::path_entry &p1, const pbrt::path_entry &p2) {
  // find the closest match between paths
  const pbrt::path_entry &s_path = p1.pathlen < p2.pathlen ? p1 : p2;
  const pbrt::path_entry &l_path = p1.pathlen >= p2.pathlen ? p1 : p2;

  float distsum = 0.f;
  for (int i = 1; i < s_path.vertices.size(); ++i) {
    float localmin = std::numeric_limits<float>::max();
    for (int j = 1; j < l_path.vertices.size(); ++j) {
    // localmin = std::min(pbrt::Vector3f(pbrt::FromArray(s_path.vertices[i].v) - pbrt::FromArray(l_path.vertices[j].v)).LengthSquared(), localmin);
      localmin = std::min(pbrt::Vector3f(pbrt::Vector3f(pbrt::FromArray(s_path.vertices[i].v) - pbrt::FromArray(s_path.vertices[i-1].v))-
      pbrt::Vector3f(pbrt::FromArray(l_path.vertices[j-1].v) - pbrt::FromArray(l_path.vertices[j].v))).LengthSquared(), localmin);
    }
    distsum += localmin;
  }
  return std::sqrt(distsum); // geometric mean ?
}


void PathDistance::getElementsToSort(std::vector<uint64_t> &sampleset) {
  if(!resortelements)
    return;

  sampleset.insert(sampleset.end(), elements.begin(), elements.end());
  elements.clear();
  resortelements = false;
  currentcost = 0;
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

std::shared_ptr<Label> PathDistanceGenerator::generateRandomCentroid() {
  uint64_t random_id(rng(generator) * paths.size());
  return std::shared_ptr<Label>(new PathDistance(paths[random_id]));}
} // namespace Kmeans
