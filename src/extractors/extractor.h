//
// Created by nispaur on 4/21/17.
//

#ifndef PBRT_V3_EXTRACTOR_H
#define PBRT_V3_EXTRACTOR_H

#include "extractors/pathoutput.h"
#include "extractors/pathio.h"
#include "reflection.h"
#include "geometry.h"
#include "film.h"
#include "pbrt.h"

namespace pbrt {

// Extractor types
enum ExtractorType : unsigned int {
    FILM_EXTRACTOR,
    PATH_EXTRACTOR,
    CUSTOM_EXTRACTOR, // TODO : find a better identifier
    EXTRACTOR_SET,
    NUM_EXTRACTOR_TYPE
};

#define NEW_EXTRACTOR_INTERFACE
#ifdef NEW_EXTRACTOR_INTERFACE
// Definition of extractor interface : starting work on Sept. 13rd
class Extractor {
public:
    Extractor(ExtractorType t) : type(t) {}
    virtual ~Extractor() {}

    // type of extractor
    ExtractorType Type() const { return type; }

    // Tiling stuff
    virtual std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound) = 0;
    // End tile get a tiledExtractor as parameter and merge it with the current full extractor.
    // Tiled extractors might use this method with a null parameter to finalize the tile
    virtual void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor = nullptr) = 0;
    // todo, is mergeTile mandatory ? EndTile do the job

    // Pixel stuff
    virtual void StartPixel(Point2i pixel) {};
    virtual void EndPixel() {};
    // todo : do we need UpdatePixel ? which profile for such a method ?

    // Path (or Sample) stuff. May be simplified ?
    virtual void InitPath(const Point2f &p) {};

    // incremental building of a path, usefull for SamplerIntegrators
    virtual void StartPath(const RayDifferential &r, int depth, const Scene &scene) {}
    virtual void AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {}
    // direct building of a path, usefull for bidirectionnal integrators
    virtual void StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t) {}

    virtual void EndPath(const Spectrum &throughput, float weight = 1.f) {};

    // General stuff
    // What are the parameters of this ???
    virtual void Initialize(const Bounds3f & worldBound) {}
    virtual void Flush(float splatScale = 1.f) = 0;

private:
    ExtractorType type;

};

class ExtractorSet : public Extractor {
    // IMPORTANT : !!!!!!
    // Assume multimap contains extractors of each type in the same order ...
public:
    ExtractorSet() : Extractor(EXTRACTOR_SET) {}
    ~ExtractorSet() {}

    std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound);
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor);

    void StartPixel(Point2i pixel);
    void EndPixel();

    void InitPath(const Point2f &p);
    void StartPath(const RayDifferential &r, int depth, const Scene &scene);
    void AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf);
    void StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t);
    void EndPath(const Spectrum &throughput, float weight = 1.f);

    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale);

    void AddExtractor(std::unique_ptr<Extractor> e) {
        extractors.emplace( std::make_pair(e->Type(), std::move(e)) );
    }

private:
    ExtractorSet (const ExtractorSet &e) = delete;
    std::multimap< ExtractorType, std::unique_ptr<Extractor> > extractors;
};

// -----------------
// Normal extractor
// -----------------
// full extractor

class ExtractorNormal : public Extractor {

public:
    ExtractorNormal(Film *f) : Extractor(FILM_EXTRACTOR), film(f) {}
    ~ExtractorNormal() {}

    // Extractor interface implementation
    // Tiling stuff
    std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound);
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor);

    // General stuff
    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale);

private:
    ExtractorNormal (const ExtractorSet &e) = delete;
    std::unique_ptr<Film> film;
};

// Tile Normal extractor
class ExtractorNormalTiled : public Extractor {
public:
    ExtractorNormalTiled(Film *f, const Bounds2i &tileBound);
    ~ExtractorNormalTiled() {}

    std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound);
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor);

    void InitPath(const Point2f &p);
    void StartPath(const RayDifferential &r, int depth, const Scene &scene);
    void StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t);
    void AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf);
    void EndPath(const Spectrum &throughput, float weight = 1.f);


    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale);

    std::unique_ptr<FilmTile> GetTile() {
        return std::move(film);
    }

private:
    ExtractorNormalTiled (const ExtractorNormalTiled &e) = delete;
    std::unique_ptr<FilmTile> film;

    // State for extraction
    Point2f samplePos;
    Normal3f n;

    bool first;
    bool valid_path;
};

// -----------------
// Depth extractor
// TODO : generalize depth extractor for each kind of Camera
// -----------------
// full extractor

class ExtractorDepth : public Extractor {

public:
    ExtractorDepth(Film *f, float zn=1e-2f, float zf=1000.f);
    ~ExtractorDepth();

    std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound);
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor);

    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale);

private:
    ExtractorDepth (const ExtractorSet &e) = delete;

    std::unique_ptr<Film> film;
    const Float znear;
    const Float zfar;

};

// Tile extractor
class ExtractorDepthTiled : public Extractor {
public:
    ExtractorDepthTiled(Film *f, const Bounds2i &tileBound, float zn, float zf);
    ~ExtractorDepthTiled() {}

    std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound);
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor);


    void InitPath(const Point2f &p);
    void StartPath(const RayDifferential &r, int depth, const Scene &scene);
    void StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t);
    void AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf);
    void EndPath(const Spectrum &throughput, float weight = 1.f);


    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale);

    std::unique_ptr<FilmTile> GetTile() {
        return std::move(film);
    }
private:
    ExtractorDepthTiled (const ExtractorNormalTiled &e) = delete;
    float ConvertDepth(float z);

    std::unique_ptr<FilmTile> film;

    // State for extraction
    Point2f samplePos;

    const Float znear;
    const Float zfar;
    Float zscale;

    Point3f rayorigin;
    Float realdepth;

    bool first;
    bool valid_path;

};

// -----------------
// Albedo extractor
// TODO : generalize depth extractor for each kind of Camera
// -----------------
// full extractor

class ExtractorAlbedo : public Extractor {

public:
    ExtractorAlbedo(Film *f, BxDFType bxdfType, bool integrateAlbedo, int nbSamples);
    ~ExtractorAlbedo();

    std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound);
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor);

    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale);

private:
    ExtractorAlbedo (const ExtractorSet &e) = delete;

    std::unique_ptr<Film> film;
    BxDFType bxdf_type;
    bool integrate_albedo;
    int nb_samples;

};

// Tile extractor
class ExtractorAlbedoTiled : public Extractor {
public:
    ExtractorAlbedoTiled(Film *f, const Bounds2i &tileBound, BxDFType bxdfType, bool integrateAlbedo, int nbSamples);
    ~ExtractorAlbedoTiled() {}

    std::unique_ptr<Extractor> StartTile(const Bounds2i &tileBound);
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor);


    void InitPath(const Point2f &p);
    void StartPath(const RayDifferential &r, int depth, const Scene &scene);
    void StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t);
    void AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf);
    void EndPath(const Spectrum &throughput, float weight = 1.f);


    void Initialize(const Bounds3f &worldBound);
    void Flush(float splatScale);

    std::unique_ptr<FilmTile> GetTile() {
        return std::move(film);
    }
private:
    ExtractorAlbedoTiled (const ExtractorNormalTiled &e) = delete;
    Spectrum computeAlbedo(BSDF *bsdf, const Vector3f &dir);

    std::unique_ptr<FilmTile> film;

    // State for extraction
    Point2f samplePos;

    BxDFType bxdf_type;
    bool integrate_albedo;
    int nb_samples;
    Spectrum rho;
    std::vector<Point2f> wi;
    std::vector<Point2f> wo;

    bool first;
    bool valid_path;
};


#else

struct IntegratorStatistics {
    float luminance;
    float variance;
    float error;
    int nbsamples;
};

class Container {
public:
    Container() {};

    virtual void Init(const RayDifferential &r, int depth, const Scene &scene) =0;

    virtual void ReportData(const SurfaceInteraction &isect) {};

    virtual void ReportData(const RayDifferential &r) {};

    virtual void ReportData(const Spectrum &L) {};

    virtual void ReportData(const IntegratorStatistics &S) {};

    virtual void ReportData(const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {};

    virtual void BuildPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t) {};

    virtual void AddSplat(const Point2f &pSplat, Film *film) {};

    virtual Spectrum ToSample() const = 0;

    // FIXME: crappy solution
    virtual std::vector<path_entry> GetPaths() { return std::vector<path_entry>(); };

    virtual ~Container() {}

};

// Functor class

class ExtractorFunc {
public:
    virtual std::shared_ptr<Container> GetNewContainer(const Point2f &p) const = 0;
    virtual ExtractorType GetType() const = 0;
};

// Extractor main class
class Extractor {
public:
    Extractor(const ExtractorFunc *f, PathOutput *p) : f(f), /*film(nullptr), */ p(p) {};

    Extractor(const ExtractorFunc *f, Film *film) : f(f), film(film)/*, p(nullptr)*/ {};

    const ExtractorFunc *f;
    union {
        Film *film;
        PathOutput *p;
    };
};

class Containers {
public:
    Containers() {};

    inline void Add(std::shared_ptr<Container> container) {
        containers.push_back(container);
    }

    void Init(const RayDifferential &r, int depth, const Scene &scene);

    template<typename T>
    void ReportData(const T &value) {
        for (std::shared_ptr<Container> c : containers)
            c->ReportData(value);
    }

    void BuildPath(const Vertex *lightVertices, const Vertex *cameraVertices, int s, int t) {
        for (std::shared_ptr<Container> c : containers)
            c->BuildPath(lightVertices, cameraVertices, s, t);
    }

    Spectrum ToSample(int id) const {
        CHECK_LT(id, containers.size());
        return containers[id]->ToSample();
    }

    std::shared_ptr<Container> &GetContainer(int id) {
        CHECK_LT(id, containers.size());
        return containers[id];
    }


    // TODO: cleaner approach
    void AddSplats(int id, const Point2f &pSplat, Film *film) const {
        CHECK_LT(id, containers.size());
        return containers[id]->AddSplat(pSplat, film);
    }

private:
    std::vector<std::shared_ptr<Container>> containers;
};

// Extractor Manager

class ExtractorTileManager {
public:

    void Add(std::unique_ptr<FilmTile> tile) {
        dispatchtable.push_back({false, filmtiles.size()});
        filmtiles.push_back(std::move(tile));
    }

    void Add(std::unique_ptr<PathOutputTile> tile) {
        dispatchtable.push_back({true, pathtiles.size()});
        pathtiles.push_back(std::move(tile));
    }

    void AddSamples(const Point2f &pFilm, std::unique_ptr<Containers> container, Float sampleWeight = 1.f);


    std::unique_ptr<PathOutputTile> GetPathTile(int id) {
        return std::unique_ptr<PathOutputTile>(std::move(pathtiles[dispatchtable[id].second]));
    }

    std::unique_ptr<FilmTile> GetFilmTile(int id) {
        return std::unique_ptr<FilmTile>(std::move(filmtiles[dispatchtable[id].second]));
    }

private:
    std::vector<std::pair<bool, int>> dispatchtable;
    //std::multimap<ExtractorType, int> dispatchtable;
    std::vector<std::unique_ptr<FilmTile>> filmtiles;
    std::vector<std::unique_ptr<PathOutputTile>> pathtiles;
};

class ExtractorManager {
public:
    ExtractorManager() {};

    void Add(Extractor *extractor) {
        if (extractor->film != nullptr) {
            dispatchtable.push_back({false, films.size()});
            films.push_back(extractor->film);
        } else {
            dispatchtable.push_back({true, paths.size()});
            paths.push_back(extractor->p);
        }

        extractors.push_back(extractor);
    }

    std::unique_ptr<Containers> GetNewContainer(const Point2f &p);

    std::unique_ptr<ExtractorTileManager> GetNewExtractorTile(const Bounds2i &sampleBounds);

    void MergeTiles(std::unique_ptr<ExtractorTileManager> tiles);

    void WriteOutput(Float splatScale = 1);

    void AddSplats(const Point2f &pSplat, const Containers &container);

private:
    std::vector<Extractor *> extractors;
    std::vector<std::pair<bool, int>> dispatchtable;
    std::vector<Film *> films;
    std::vector<PathOutput *> paths;
};



// Statistic extractor

class StatisticsContainer : public Container {
public:
    StatisticsContainer(const Point2f &pFilm) : p(pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &scene);

    void ReportData(const IntegratorStatistics &stats);

    Spectrum ToSample() const;

private:
    const Point2f p;

    IntegratorStatistics statistics;
};

class StatisticsExtractor : public ExtractorFunc {
public:
    std::shared_ptr<Container> GetNewContainer(const Point2f &p) const;
    ExtractorType GetType() const { return ExtractorType::CUSTOM_EXTRACTOR; }
};

#endif

// API Methods
Extractor *CreateNormalExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera);

Extractor *CreateDepthExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera);

Extractor *CreateAlbedoExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera);

Extractor *CreateStatisticsExtractor(const ParamSet &params, const Point2i &fullResolution,
                                     Float diagonal, const std::string &imageFilename);

}


#endif //PBRT_V3_EXTRACTOR_H
