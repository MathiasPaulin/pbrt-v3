//
// Created by Mathias Paulin on 20/09/2017.
//

#ifndef PBRT_V3_STATISTICEXTRACTOR_H
#define PBRT_V3_STATISTICEXTRACTOR_H

#include "extractors/extractor.h"
namespace pbrt {

struct PixelStatistics {
    float luminance_mean = 0.;
    float luminance_variance = 0.;
    float luminance_error = 0.;
    int nbsamples = 0;
};

class PixelStatisticsStorageTile {
public:
    explicit PixelStatisticsStorageTile(const Bounds2i &tileBound);
    PixelStatisticsStorageTile(const PixelStatisticsStorageTile &other) = delete;
    ~PixelStatisticsStorageTile() = default;

    void Accumulate(const Point2i &pixel, const Spectrum &L);

    void Clear();

    Bounds2i GetBounds() const { return limits; }

    PixelStatistics &GetPixel(const Point2i &pixel) {
        return pixel_stats[PixelToIndex(pixel)];
    }

private:
    int PixelToIndex(const Point2i &pixel) const {
        CHECK(InsideExclusive(pixel, limits));
        return (pixel.x - limits.pMin.x) + (pixel.y - limits.pMin.y) * (limits.pMax.x - limits.pMin.x);
    }

    const Bounds2i limits;
    std::vector<PixelStatistics> pixel_stats;
};

class PixelStatisticsStorage {
public:
    explicit PixelStatisticsStorage(const Point2i &dim);
    PixelStatisticsStorage(const PixelStatisticsStorage &other) = delete;
    ~PixelStatisticsStorage() = default;

    Bounds2i GetBounds() const { return limits; }

    void MergeTile(PixelStatisticsStorageTile &tile);

    void Clear();

    PixelStatistics &GetPixel(const Point2i &pixel) {
        return pixel_stats[PixelToIndex(pixel)];
    }

private:
    int PixelToIndex(const Point2i &pixel) const {
        CHECK(InsideExclusive(pixel, limits));
        return (pixel.x - limits.pMin.x) + (pixel.y - limits.pMin.y) * (limits.pMax.x - limits.pMin.x);
    }

    const Bounds2i limits;
    std::vector<PixelStatistics> pixel_stats;
    std::mutex mutex;
};


// Tile extractor
class ExtractorStatisticsTile : public Extractor {
public:
    ExtractorStatisticsTile(const Bounds2i &tileBound);

    ExtractorStatisticsTile(const ExtractorNormalTile &e) = delete;

    ~ExtractorStatisticsTile() = default;

    std::unique_ptr<Extractor> BeginTile(const Bounds2i &tileBound) const;

    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const;

    void BeginPixel(const Point2i &pix);

    void EndPixel();

    // Sample stuff
    void BeginSample(const Point2f &p);
    void EndSample(const Spectrum &throughput, float weight = 1.f);


    void Initialize(const Bounds3f &worldBound);

    void Flush(float splatScale);

    PixelStatisticsStorageTile &GetTile(){
        return pixel_statistics;
    }

private:

    PixelStatisticsStorageTile pixel_statistics;
    Point2i current_pixel;
    bool valid_luminance;
    bool valid_pixel;
};


class ExtractorStatistics : public Extractor {

public:
    ExtractorStatistics(const std::string &filesuffix, const Point2i &resolution, float diagonal);
    ExtractorStatistics() = delete;
    ExtractorStatistics(const ExtractorSet &e) = delete;
    ~ExtractorStatistics() = default;

    std::unique_ptr <Extractor> BeginTile(const Bounds2i &tileBound) const;

    void EndTile(std::unique_ptr <Extractor> sourceTiledExtractor) const;

    void Initialize(const Bounds3f &worldBound);

    void Flush(float splatScale);

private:
    mutable PixelStatisticsStorage pixel_statistics;
    std::unique_ptr <Film> luminance_error_film;
    std::unique_ptr <Film> luminance_mean_film;
    std::unique_ptr <Film> nbsamples_film;

    const Bounds2i limits;
};

}

#endif //PBRT_V3_STATISTICEXTRACTOR_H
