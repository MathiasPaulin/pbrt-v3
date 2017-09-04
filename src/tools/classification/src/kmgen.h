#ifndef PBRT_V3_KMEANS_H
#define PBRT_V3_KMEANS_H

#include <functional>
#include "tools/pathtool.h"
// PBRT Geometry tools
#include "core/geometry.h"
#include <random>
#include <memory>

namespace Kmeans {

class Label {
  public:
    Label() {}

    virtual float distance(const pbrt::path_entry &p) const = 0;

    virtual void recompute_centroid() = 0;

    void label_element(const pbrt::path_entry &p, uint64_t path_id) {
      update_mean(p);
      elements.push_back(path_id);
    }

    size_t size() const { return elements.size(); }

    std::vector<uint64_t> elements;

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
    Classifier(int k, PathFile &f, std::shared_ptr<CentroidGenerator> g, int maxiterations = -1) :
            k(k), paths(f), maxiterations(maxiterations), iteration(0), generator(g) {}

    void run();

    std::vector<std::shared_ptr<Label>> getLabels();

  private:

    void sortElement(int i);

    void recalculateCentroids();

    bool end();

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

    float distance(const pbrt::path_entry &p) const;

    void recompute_centroid();


  private:
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

} // namespace Kmeans

#endif //PBRT_V3_KMEANS_H
