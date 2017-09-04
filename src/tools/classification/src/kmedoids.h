#ifndef PBRT_V3_KMEDOIDS_H
#define PBRT_V3_KMEDOIDS_H

#include <functional>
#include "tools/pathtool.h"
// PBRT Geometry tools
#include "core/geometry.h"
#include "core/parallel.h"
#include <random>
#include <memory>
#include <set>

namespace Kmedoids {

class Label {
  public:
    Label(bool resortelements = false) : resortelements(resortelements) {}

    virtual float distance(const pbrt::path_entry &p) = 0;

    virtual void recompute_centroid(const PathFile &p) = 0;

    void label_element(const pbrt::path_entry &p, uint64_t path_id) {
      update_mean(p);
      elements.push_back(path_id);
    }

    virtual bool operator ==(const Label &b) const = 0;

    size_t size() const { return elements.size(); }
    size_t cost() const { return currentcost; }
    bool fixed_medoid() const { return !resortelements && !elements.empty(); }

    std::vector<uint64_t> elements;
    virtual void getElementsToSort(std::vector<uint64_t> &sampleset) = 0;
  protected:
    size_t currentcost;
    bool resortelements;

  private:
    virtual void update_mean(const pbrt::path_entry &p) = 0;
};


class CentroidGenerator {
  public:
    CentroidGenerator(const PathFile &p) : paths(p) {
    }

    virtual std::shared_ptr<Label> generateRandomCentroid() = 0;

  protected:
    PathFile paths;
    std::uniform_real_distribution<float> rng;
    std::default_random_engine generator;

  private:
};

class Classifier {
  public:
    Classifier(int k, PathFile &f, std::shared_ptr<CentroidGenerator> g, int samplesize, int maxiterations = -1) :
            k(k), paths(f), maxiterations(maxiterations), samplesize(samplesize), iteration(0), generator(g) {
      pbrt::ParallelInit();
    }

    void run();

    std::vector<std::shared_ptr<Label>> getLabels();

    ~Classifier() {
      pbrt::ParallelCleanup();
    }

  private:

    void sortElement(int i);

    void recalculateCentroids();

    bool end();

    size_t lastcost;
    size_t cost;
    int samplesize;
    std::vector<uint64_t> sampleset;
    std::shared_ptr<CentroidGenerator> generator;
    PathFile paths;
    int k;
    int maxiterations;
    int iteration;
    std::vector<std::shared_ptr<Label>> labels;
};

// Test class; euclidean distance in 3D space

class DistanceLabel : public Label {
  public:
    DistanceLabel(const pbrt::Vector3f &centroid) : centroid(centroid) {
      length = centroid.Length();
      std::cerr << "New Distance Label generated; length = " << length << std::endl;
    }

    float distance(const pbrt::path_entry &p);
    void recompute_centroid(const PathFile &p);
    void getElementsToSort(std::vector<uint64_t> &sampleset);
    bool operator ==(const DistanceLabel &b) const {
      return b.centroid == centroid;
    }

    bool operator ==(const Label &b) const {
      const DistanceLabel *label_ptr = dynamic_cast<const DistanceLabel *>(&b);
      return (label_ptr == nullptr) ? false : (*label_ptr == *this);
    }

  private:
    float distance(const pbrt::path_entry &p, float centroidlength) const;
    void update_mean(const pbrt::path_entry &p);

    pbrt::Vector3f centroid;
    float length;
    float meanlength;
    float sigma_sq; // Variance
};

class DistanceGenerator : public CentroidGenerator {
  public:
    DistanceGenerator(const PathFile &p);

    std::shared_ptr<Label> generateRandomCentroid();

  private:
    pbrt::Bounds3f b;
};

// Test class; Levenshtein distance between path notations

class LevenshteinDistance : public Label {
  public:
    LevenshteinDistance(const std::string &centroid) : centroid(centroid) {
      std::cerr << "New Distance Label generated; string = " << centroid << std::endl;
    }

    float distance(const pbrt::path_entry &p);

    bool operator ==(const Label &b) const {
      const LevenshteinDistance *label_ptr = dynamic_cast<const LevenshteinDistance *>(&b);
      return (label_ptr == nullptr) ? false : (*label_ptr == *this);
    }

    bool operator ==(const LevenshteinDistance &b) const {
      return b.centroid == centroid;
    }

    void recompute_centroid(const PathFile &p);
    void getElementsToSort(std::vector<uint64_t> &elements);
    void update_mean(const pbrt::path_entry &p);

  private:
    int distance(const std::string &s1, const std::string &s2) const;
    std::string centroid;
    // uint64_t distsum;
    uint64_t last_distance;
    float length;
    float meanlength;
    float sigma_sq; // Variance
};

class LevenshteinGenerator : public CentroidGenerator {
  public:
    LevenshteinGenerator(const PathFile &p) : CentroidGenerator(p) {}

    std::shared_ptr<Label> generateRandomCentroid();

  private:

};

class PathDistance : public Label {
  public:
    PathDistance(const pbrt::path_entry &centroid) : centroid(centroid) {
      std::cerr << "New PathDistance label generated; path " << centroid.path << std::endl;
    }

    float distance(const pbrt::path_entry &p);

    bool operator ==(const Label &b) const {
      const PathDistance *label_ptr = dynamic_cast<const PathDistance *>(&b);
      return (label_ptr == nullptr) ? false : (*label_ptr == *this);
    }

    bool operator ==(const PathDistance &b) const {
      return b.centroid == centroid;
    }

    void recompute_centroid(const PathFile &p);
    void getElementsToSort(std::vector<uint64_t> &elements);
    void update_mean(const pbrt::path_entry &p);

  private:
    float distance(const pbrt::path_entry &p1, const pbrt::path_entry &p2);

    pbrt::path_entry centroid;
    float last_distance;
    float length;
    float meanlength;
    float sigma_sq; // Variance
};

class PathDistanceGenerator : public CentroidGenerator {
  public:
    PathDistanceGenerator(const PathFile &p) : CentroidGenerator(p) {}

    std::shared_ptr<Label> generateRandomCentroid();

  private:
};

} // namespace Kmeans

#endif //PBRT_V3_KMEDOIDS_H
