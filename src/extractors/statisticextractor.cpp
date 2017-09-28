//
// Created by Mathias Paulin on 20/09/2017.
//

#include "statisticextractor.h"
#include "paramset.h"
#include "spectrum.h"
#include "camera.h"
#include "filters/box.h"

namespace pbrt {
/*
 * Add below extractor for statistic about the convergence
 */

PixelStatisticsStorageTile::PixelStatisticsStorageTile(const Bounds2i &tileBound) : limits(tileBound), pixel_stats(limits.Area()) {
}


void PixelStatisticsStorageTile::Accumulate(const Point2i &pixel, const Spectrum &L) {

    if (InsideExclusive(pixel, limits)) {
        PixelStatistics &buf = GetPixel(pixel);
        /* TODO : Compute the increments on buf.nbsamples, buf.luminance_mean, buf.luminance_variance */
        float sample_luminance = L.y();
        ++buf.nbsamples;
        float delta = sample_luminance - buf.luminance_mean;
        buf.luminance_mean += delta / buf.nbsamples;
        buf.luminance_variance += delta * (sample_luminance - buf.luminance_mean);
    }

}

void PixelStatisticsStorageTile::Clear() {
    std::for_each(
            pixel_stats.begin(),
            pixel_stats.end(),
            [](PixelStatistics &p) {
                p.luminance_mean = p.luminance_error = p.luminance_variance = 0.f;
                p.nbsamples = 0;
            });
}

PixelStatisticsStorage::PixelStatisticsStorage(const Point2i &dim) : limits(Point2i(), dim), pixel_stats(limits.Area()) {

}


void PixelStatisticsStorage::MergeTile(PixelStatisticsStorageTile &tile) {
    std::lock_guard<std::mutex> lock(mutex);

    for (Point2i p : tile.GetBounds()) {
        PixelStatistics &buf_source = tile.GetPixel(p);
        PixelStatistics &buf = GetPixel(p);
        /* TODO : set buf to finalized statistics from buf_source */
        buf.nbsamples = buf_source.nbsamples;
        buf.luminance_mean = buf_source.luminance_mean;
        buf.luminance_variance = buf_source.luminance_variance / (buf.nbsamples - 1);
        buf.luminance_error = std::sqrt(buf.luminance_variance / buf.nbsamples);

    }
}

void PixelStatisticsStorage::Clear() {
    for(Point2i p : limits) {
        PixelStatistics &pixel = GetPixel(p);
        pixel.luminance_mean = pixel.luminance_error = pixel.luminance_variance = 0.f;
        pixel.nbsamples = 0;
    }
}


ExtractorStatisticsTile::ExtractorStatisticsTile(const Bounds2i &tileBound) :
        Extractor(CUSTOM_EXTRACTOR), pixel_statistics(tileBound) {
    valid_luminance = valid_pixel = false;
}

std::unique_ptr <Extractor> ExtractorStatisticsTile::BeginTile(const Bounds2i &tileBound) const {
    return std::unique_ptr<Extractor>(nullptr);
}

void ExtractorStatisticsTile::EndTile(std::unique_ptr <Extractor> sourceTiledExtractor) const {
    // Nothing to do here
}

void ExtractorStatisticsTile::BeginPixel(const Point2i &pix) {

    valid_pixel = InsideExclusive(pix, pixel_statistics.GetBounds());
    if (valid_pixel)
        current_pixel = pix;

}

void ExtractorStatisticsTile::EndPixel() {
    // nothing special to do here
    valid_pixel = false;
}

void ExtractorStatisticsTile::BeginSample(const Point2f &p) {
    if (valid_pixel)
        valid_luminance = InsideExclusive((Point2i) p, pixel_statistics.GetBounds());
}

void ExtractorStatisticsTile::EndSample(const Spectrum &throughput, float weight) {
    if (valid_luminance && valid_pixel) {
        pixel_statistics.Accumulate(current_pixel, throughput * weight);
    }
    valid_luminance = false;
}

void ExtractorStatisticsTile::Initialize(const Bounds3f &worldBound) {

}

void ExtractorStatisticsTile::Flush(float splatScale) {

}

ExtractorStatistics::ExtractorStatistics(const std::string &filesuffix,
                                         const Point2i &resolution,
                                         float diagonal) : Extractor(CUSTOM_EXTRACTOR), pixel_statistics(resolution), limits(Point2i(), resolution) {
    luminance_mean_film =  std::unique_ptr <Film>(new Film(
            resolution,
            Bounds2f(Point2f(0, 0), Point2f(1, 1)),
            std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
            diagonal, "luminance"+filesuffix, 1.f));

    luminance_error_film =  std::unique_ptr <Film>(new Film(
            resolution,
            Bounds2f(Point2f(0, 0), Point2f(1, 1)),
            std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
            diagonal, "error"+filesuffix, 1.f));

    nbsamples_film =  std::unique_ptr <Film>(new Film(
            resolution,
            Bounds2f(Point2f(0, 0), Point2f(1, 1)),
            std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
            diagonal, "nbsamples"+filesuffix, 1.f));
}


std::unique_ptr <Extractor> ExtractorStatistics::BeginTile(const Bounds2i &tileBound) const {
    Bounds2i tilePixelBounds = Intersect(tileBound, limits);
    return std::unique_ptr<Extractor>(new ExtractorStatisticsTile(tilePixelBounds));
}

void ExtractorStatistics::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    auto source = static_cast<ExtractorStatisticsTile *>(sourceTiledExtractor.get());
    pixel_statistics.MergeTile(source->GetTile());
}

void ExtractorStatistics::Initialize(const Bounds3f &worldBound) {

}

void ExtractorStatistics::Flush(float splatScale) {

    for (const auto &p : pixel_statistics.GetBounds()) {
        Point2f pf(p.x, p.y);
        PixelStatistics & stat = pixel_statistics.GetPixel(p);
        luminance_error_film->AddSplat(pf, Spectrum(stat.luminance_error));
        luminance_mean_film->AddSplat(pf, Spectrum(stat.luminance_mean));
        nbsamples_film->AddSplat(pf, Spectrum(stat.nbsamples));
    }

    luminance_error_film->WriteImage(1.0f);
    luminance_mean_film->WriteImage(1.0f);
    nbsamples_film->WriteImage(1.0f);

}

/***************************************************************************************************
 *
 ***************************************************************************************************/

Extractor *CreateStatisticsExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera) {
    std::string filesuffix = params.FindOneString("outputfile", "");
    if (filesuffix == "") filesuffix = camera->film->filename;

    return new ExtractorStatistics(filesuffix, camera->film->fullResolution, camera->film->diagonal);

}

}
