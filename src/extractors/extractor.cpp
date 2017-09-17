//
// Created by nispaur on 4/21/17.
//

#include <integrators/bdpt.h>
#include "filters/box.h"
#include "scene.h"
#include "paramset.h"
#include "interaction.h"
#include "extractors/extractor.h"
#include "extractors/pathoutput.h"
#include "spectrum.h"
#include "camera.h"

namespace pbrt {

#ifdef NEW_EXTRACTOR_INTERFACE
// TODO Verify all profilePhase ....
// ------------
// ExtractorSet
// ------------
std::unique_ptr<Extractor> ExtractorSet::StartTile(const Bounds2i &tileBound) {
    ExtractorSet *tileSet = new ExtractorSet;
    for (const auto &e : extractors)
        tileSet->AddExtractor(e.second->StartTile(tileBound));
    return std::unique_ptr<Extractor>(tileSet);
}

void ExtractorSet::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) {
    // TODO merge tile (source in e) in the current extractor set (target)
    CHECK( sourceTiledExtractor->Type() == EXTRACTOR_SET );
    ExtractorSet *source = static_cast<ExtractorSet *>(sourceTiledExtractor.get());
    // merge film extractor tiles
    {
        auto targetTiles = extractors.equal_range(FILM_EXTRACTOR);
        auto sourceTiles = source->extractors.equal_range(FILM_EXTRACTOR);
        auto sourceIt = sourceTiles.first;
        for (auto targetIt = targetTiles.first; targetIt != targetTiles.second; ++targetIt, ++sourceIt)
            targetIt->second->EndTile(std::move(sourceIt->second));
    }
    // merge path extractor tiles
    {
        auto targetTiles = extractors.equal_range(PATH_EXTRACTOR);
        auto sourceTiles = source->extractors.equal_range(PATH_EXTRACTOR);
        auto sourceIt = sourceTiles.first;
        for (auto targetIt = targetTiles.first; targetIt != targetTiles.second; ++targetIt, ++sourceIt)
            targetIt->second->EndTile(std::move(sourceIt->second));
    }
    // merge custom extractor tiles
    {
        auto targetTiles = extractors.equal_range(CUSTOM_EXTRACTOR);
        auto sourceTiles = source->extractors.equal_range(CUSTOM_EXTRACTOR);
        auto sourceIt = sourceTiles.first;
        for (auto targetIt = targetTiles.first; targetIt != targetTiles.second; ++targetIt, ++sourceIt)
            targetIt->second->EndTile(std::move(sourceIt->second));
    }
    // merge set extractor tiles
    {
        auto targetTiles = extractors.equal_range(EXTRACTOR_SET);
        auto sourceTiles = source->extractors.equal_range(EXTRACTOR_SET);
        auto sourceIt = sourceTiles.first;
        for (auto targetIt = targetTiles.first; targetIt != targetTiles.second; ++targetIt, ++sourceIt)
            targetIt->second->EndTile(std::move(sourceIt->second));
    }
}

// Pixel stuff
void ExtractorSet::StartPixel(Point2i pixel) {
    for (const auto &e : extractors)
        e.second->StartPixel(pixel);
}

void ExtractorSet::EndPixel()  {
    for (const auto &e : extractors)
        e.second->EndPixel();
}

// Path (or Sample) stuff. May be simplified ?
void ExtractorSet::InitPath(const Point2f &p)  {
    for (const auto &e : extractors)
        e.second->InitPath(p);
}

// incremental building of a path, usefull for SamplerIntegrators
void ExtractorSet::StartPath(const RayDifferential &r, int depth, const Scene &scene) {
    for (const auto &e : extractors)
        e.second->StartPath(r, depth,scene);
}

void ExtractorSet::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    for (const auto &e : extractors)
        e.second->AddPathVertex(isect, bsdf);
}

// direct building of a path, usefull for bidirectionnal integrators
void ExtractorSet::StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t) {
    for (const auto &e : extractors)
        e.second->StartPath(lightVertrices, cameraVertrices, s, t);
}

void ExtractorSet::EndPath(const Spectrum &throughput, float weight){
    for (const auto &e : extractors)
        e.second->EndPath(throughput, weight);
}

// General stuff
// What are the parameters of this ???
void ExtractorSet::Initialize(const Bounds3f &worldBound) {
    for (const auto &e : extractors)
        e.second->Initialize(worldBound);
}

void ExtractorSet::Flush(float splatScale) {
    for (const auto &e : extractors)
        e.second->Flush(splatScale);
}



// ---------------------------------------------------------
// Normal extractor
// ---------------------------------------------------------

// Extractor interface implementation
// Tiling stuff
std::unique_ptr<Extractor> ExtractorNormal::StartTile(const Bounds2i &tileBound) {
    // generate an ExtractorNormalTile, it and return it
    return std::unique_ptr<Extractor>(new ExtractorNormalTiled(film.get(),tileBound));
}

void ExtractorNormal::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) {
    // Verify that e is an ExtractornormalTile and merge it with current film
    // well, for the moment, we assume it is the case (due to propertiez of multimap)
    ExtractorNormalTiled *source = static_cast<ExtractorNormalTiled *>(sourceTiledExtractor.get());
    film->MergeFilmTile(source->GetTile());
}

// General stuff
// What are the parameters of this ???
void ExtractorNormal::Initialize(const Bounds3f &worldBound){
    ProfilePhase pp(Prof::ExtractorInit);
}

void ExtractorNormal::Flush(float splatScale){
    // here, write the film to the file
    film->WriteImage(splatScale);
}


ExtractorNormalTiled::ExtractorNormalTiled(Film *f, const Bounds2i &tileBound) : Extractor(FILM_EXTRACTOR){
    ProfilePhase pp(Prof::ExtractorInit);
    film =f->GetFilmTile(tileBound);
}


std::unique_ptr<Extractor> ExtractorNormalTiled::StartTile(const Bounds2i &tileBound) {
    return std::unique_ptr<Extractor>(this);
}

void ExtractorNormalTiled::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor){
    // Nothing to do here
}


// Path (or Sample) stuff. May be simplified ?
void ExtractorNormalTiled::InitPath(const Point2f &p) {
    samplePos = p;
    valid_path = false;
    first = true;
}
// incremental building of a path, usefull for SamplerIntegrators
void ExtractorNormalTiled::StartPath(const RayDifferential &r, int depth, const Scene &scene){
    first = (depth==0);
    valid_path = !first;
}

void ExtractorNormalTiled::StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t) {
    // TODO : if t==1 BDPT use addSplat on the entire film. find a way to manage extractors for bdpt ...
    valid_path = (t > 1);
    if (valid_path) {
        const Vertex &firstHit = cameraVertrices[1];
        if (firstHit.type == VertexType::Surface) {
            n = Faceforward(firstHit.si.n, firstHit.si.wo);
            first = false;
        } else
            valid_path = false;
    }
}

void ExtractorNormalTiled::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    ProfilePhase p(Prof::ExtractorReport);
    if (first) {
        n = Faceforward(isect.n, isect.wo);
        first = false;
        valid_path = true;
    }
}

void ExtractorNormalTiled::EndPath(const Spectrum &throughput, float weight) {
    (void) throughput;
    (void) weight;
    if (valid_path) {
        Float rgb[3] = {(n.x * 0.5f) + 0.5f, (n.y * .5f) + 0.5f, (n.z * .5f) + .5f};
        film->AddSample(samplePos, RGBSpectrum::FromRGB(rgb), 1.0);
        valid_path = false;
    }
}

// General stuff
// What are the parameters of this ???
void ExtractorNormalTiled::Initialize(const Bounds3f & worldBound) {
    ProfilePhase pp(Prof::ExtractorInit);
}
void ExtractorNormalTiled::Flush(float splatScale) {
    (void)splatScale;
}


Extractor *CreateNormalExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera) {

    std::string filename = params.FindOneString("outputfile", "");
    if (filename == "") filename = "normal_" + camera->film->filename;

    return new ExtractorNormal( new Film(
            camera->film->fullResolution,
            Bounds2f(Point2f(0, 0), Point2f(1, 1)),
            std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
            camera->film->diagonal, filename , 1.f) );
}

// ---------------------------------------------------------
// Depth extractor
// TODO : generalize depth extractor for each kind of Camera
// ---------------------------------------------------------

ExtractorDepth::ExtractorDepth(Film *f, float zn, float zf) : Extractor(FILM_EXTRACTOR), film(f), znear(zn), zfar(zf) {

}

ExtractorDepth::~ExtractorDepth() {

}

// Extractor interface implementation
// Tiling stuff
std::unique_ptr<Extractor> ExtractorDepth::StartTile(const Bounds2i &tileBound) {
    // generate an ExtractorNormalTile, it and return it
    return std::unique_ptr<Extractor>(new ExtractorDepthTiled(film.get(),tileBound, znear, zfar));
}

void ExtractorDepth::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) {
    // Verify that e is an ExtractornormalTile and merge it with current film
    // well, for the moment, we assume it is the case (due to propertiez of multimap)
    ExtractorDepthTiled *source = static_cast<ExtractorDepthTiled *>(sourceTiledExtractor.get());
    film->MergeFilmTile(source->GetTile());
}

// General stuff
// What are the parameters of this ???
void ExtractorDepth::Initialize(const Bounds3f & worldBound){
    ProfilePhase pp(Prof::ExtractorInit);
}

void ExtractorDepth::Flush(float splatScale){
    // here, write the film to the file
    film->WriteImage(splatScale);
}



ExtractorDepthTiled::ExtractorDepthTiled(Film *f, const Bounds2i &tileBound, float zn, float zf) :
        Extractor(FILM_EXTRACTOR),
        znear(zn), zfar(zf),
        realdepth(0.f) {
    ProfilePhase pp(Prof::ExtractorInit);
    film =f->GetFilmTile(tileBound);
    //zscale = ((zfar == znear) ? 1.f : znear / (znear - zfar));
    zscale = ( (zfar <= znear) ? 1.f : (zfar / (zfar - znear)) );
}

std::unique_ptr<Extractor> ExtractorDepthTiled::StartTile(const Bounds2i &tileBound) {
    return std::unique_ptr<Extractor>(this);
}

void ExtractorDepthTiled::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor){
    // Nothing to do here
}

void ExtractorDepthTiled::InitPath(const Point2f &p) {
    samplePos = p;
    valid_path = false;
}

float ExtractorDepthTiled::ConvertDepth(float z){
    //float d = zfar == 0.f ? znear / z : (-zfar * zscale) * (1.f / z) + zscale;
    float d = ( (z <= znear) ? 0.f : zscale * (1.0f - znear/z) );
    return d;
}

void ExtractorDepthTiled::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    ProfilePhase p(Prof::ExtractorReport);
    // TODO: compute z in screen space
    if ( first ) {
        float z = Vector3f(isect.p - rayorigin).Length();
        realdepth = ConvertDepth(z);
        first = false;
        valid_path = true;
    }
}

void ExtractorDepthTiled::StartPath(const RayDifferential &r, int depth, const Scene &scene){
    first = (depth==0);
    valid_path = !first;
    if (first)
        rayorigin = r.o;
}

void ExtractorDepthTiled::StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t){
    valid_path = (t>1);
    if (valid_path) {
        const Vertex &pathOrigin = cameraVertrices[0];
        const Vertex &firstHit = cameraVertrices[1];
        if (firstHit.type == VertexType::Surface) {
            rayorigin = pathOrigin.ei.p;
            float z = Vector3f(firstHit.si.p - rayorigin).Length();
            realdepth = ConvertDepth(z);
            valid_path = true;
        } else
            valid_path = false;
    }
}

void ExtractorDepthTiled::EndPath(const Spectrum &throughput, float weight){
    (void)throughput;
    (void)weight;
    if (valid_path) {
        film->AddSample(samplePos, Spectrum(realdepth), 1.0);
        valid_path = false;
    }
}

void ExtractorDepthTiled::Initialize(const Bounds3f &worldBound) {
    ProfilePhase pp(Prof::ExtractorInit);
}

void ExtractorDepthTiled::Flush(float splatScale) {
    (void)splatScale;
}

Extractor *CreateDepthExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera) {
    std::string filename = params.FindOneString("outputfile", "");
    if (filename == "") filename = "depth_" + camera->film->filename;

    Float znear = params.FindOneFloat("znear", 1e-2f);
    Float zfar = params.FindOneFloat("zfar", 10000.f);

    return new ExtractorDepth(
            new Film(
            camera->film->fullResolution,
            Bounds2f(Point2f(0, 0), Point2f(1, 1)),
            std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
            camera->film->diagonal, filename, 1.f), znear, zfar) ;
}


// ---------------------------------------------------------
// Albedo extractor
// ---------------------------------------------------------

ExtractorAlbedo::ExtractorAlbedo(Film *f, BxDFType bxdfType, bool integrateAlbedo, int nbSamples) :
        Extractor(FILM_EXTRACTOR), film(f),
        bxdf_type(bxdfType), integrate_albedo(integrateAlbedo), nb_samples(nbSamples)
{

}

ExtractorAlbedo::~ExtractorAlbedo() {

}

// Extractor interface implementation
// Tiling stuff
std::unique_ptr<Extractor> ExtractorAlbedo::StartTile(const Bounds2i &tileBound) {
    // generate an ExtractorNormalTile, it and return it
    return std::unique_ptr<Extractor>(new ExtractorAlbedoTiled(film.get(),tileBound, bxdf_type, integrate_albedo, nb_samples));
}

void ExtractorAlbedo::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) {
    // Verify that e is an ExtractornormalTile and merge it with current film
    // well, for the moment, we assume it is the case (due to propertiez of multimap)
    ExtractorAlbedoTiled *source = static_cast<ExtractorAlbedoTiled *>(sourceTiledExtractor.get());
    film->MergeFilmTile(source->GetTile());
}

// General stuff
// What are the parameters of this ???
void ExtractorAlbedo::Initialize(const Bounds3f &worldBound){
    ProfilePhase pp(Prof::ExtractorInit);
}

void ExtractorAlbedo::Flush(float splatScale){
    // here, write the film to the file
    film->WriteImage(splatScale);
}





ExtractorAlbedoTiled::ExtractorAlbedoTiled(Film *f, const Bounds2i &tileBound, BxDFType bxdfType, bool integrateAlbedo, int nbSamples) :
        Extractor(FILM_EXTRACTOR),
        bxdf_type(bxdfType), integrate_albedo(integrateAlbedo), nb_samples(nbSamples)
{
    ProfilePhase pp(Prof::ExtractorInit);

    film =f->GetFilmTile(tileBound);
    if (integrate_albedo) {
        // Static Random Number Generator; same instance across containers
        //static
        RNG rng(0);

        // Generate sample points for albedo calculation
        for (int i = 0; i < nb_samples; ++i) {
            Float x = rng.UniformFloat();
            Float y = rng.UniformFloat();
            wi.push_back(Point2f(x, y));
            wo.push_back(Point2f(x, x));
        }
    }
}

std::unique_ptr<Extractor> ExtractorAlbedoTiled::StartTile(const Bounds2i &tileBound) {
    return std::unique_ptr<Extractor>(this);
}

void ExtractorAlbedoTiled::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor){
    // Nothing to do here
}

void ExtractorAlbedoTiled::InitPath(const Point2f &p) {
    samplePos = p;
    valid_path = false;
}

void ExtractorAlbedoTiled::StartPath(const RayDifferential &r, int depth, const Scene &scene){
    first = (depth==0);
    valid_path = !first;
}


Spectrum ExtractorAlbedoTiled::computeAlbedo(BSDF *bsdf, const Vector3f &dir){
    Point2f dummy;
    return integrate_albedo ?
              bsdf->rho(nb_samples, wi.data(), wo.data(), bxdf_type) :
              bsdf->rho(dir, 0, &dummy, BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE));
}

void ExtractorAlbedoTiled::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    ProfilePhase p(Prof::ExtractorReport);
    if ( first ) {
        if (isect.bsdf) {
            rho = computeAlbedo(isect.bsdf, isect.wo);
        }
        first = false;
        valid_path = true;
    }
}

void ExtractorAlbedoTiled::StartPath(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t){
    (void)lightVertrices;
    (void)s;
    valid_path = (t>1);
    if (t>1) {
        const Vertex &firstHit = cameraVertrices[1];
        if (firstHit.type == VertexType::Surface && firstHit.si.bsdf) {
            rho = rho = computeAlbedo(firstHit.si.bsdf, firstHit.si.wo);
            valid_path = true;
        } else
            valid_path = false;
    }
}

void ExtractorAlbedoTiled::EndPath(const Spectrum &throughput, float weight){
    (void)throughput;

    if (valid_path)
        film->AddSample(samplePos, Spectrum(rho), weight);
    valid_path = false;
}

void ExtractorAlbedoTiled::Initialize(const Bounds3f &worldBound) {
    ProfilePhase pp(Prof::ExtractorInit);
}

void ExtractorAlbedoTiled::Flush(float splatScale) {
    (void)splatScale;
}


Extractor *CreateAlbedoExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera) {
    std::string filename = params.FindOneString("outputfile", "");
    if (filename == "") filename = "albedo_" + camera->film->filename;

    bool integrateAlbedo = !params.FindOneBool("closedformonly", false);
    BxDFType type;

    type = params.FindOneBool("bsdftransmission", false) ? BSDF_TRANSMISSION : BSDF_REFLECTION;
    std::string bxdftype = params.FindOneString("bxdftype", "");

    if (bxdftype == "diffuse") {
        type = BxDFType(BSDF_DIFFUSE | type);
    } else if (bxdftype == "specular") {
        type = BxDFType(BSDF_SPECULAR | type);
    } else {
        type = BxDFType(BSDF_ALL);
    }

    int nbSamples = integrateAlbedo ? params.FindOneInt("samples", 10) : 0;

    return new ExtractorAlbedo(new Film(
            camera->film->fullResolution,
            Bounds2f(Point2f(0, 0), Point2f(1, 1)),
            std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
            camera->film->diagonal, filename, 1.f), type, integrateAlbedo, nbSamples);
}
#else

void StatisticsContainer::Init(const RayDifferential &r, int depth, const Scene &scene) {
    (void)depth;
    statistics.luminance = 0.;
    statistics.variance = 0.;
    statistics.error = 0.;
    statistics.nbsamples = 0;

}

void StatisticsContainer::ReportData(const IntegratorStatistics &stats) {
    statistics = stats;
}

Spectrum StatisticsContainer::ToSample() const {
    return Spectrum(statistics.variance);
}


std::shared_ptr<Container> StatisticsExtractor::GetNewContainer(const Point2f &p) const {
    return std::shared_ptr<Container>(new StatisticsContainer(p));
}


Extractor *CreateStatisticsExtractor(const ParamSet &params, const Point2i &fullResolution,
                                     Float diagonal, const std::string &imageFilename) {
    std::string filename = params.FindOneString("outputfile", "");
    if (filename == "") filename = "error_" + imageFilename;

    return new Extractor(new StatisticsExtractor(), new Film(
            fullResolution,
            Bounds2f(Point2f(0, 0), Point2f(1, 1)),
            std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
            diagonal, filename, 1.f));
}


std::unique_ptr<Containers> ExtractorManager::GetNewContainer(const Point2f &p) {
    Containers *container = new Containers();
    for (Extractor *ext : extractors) {
        container->Add(ext->f->GetNewContainer(p));
    }

    return std::unique_ptr<Containers>(container);
}

std::unique_ptr<ExtractorTileManager> ExtractorManager::GetNewExtractorTile(const Bounds2i &tileBounds) {
    ExtractorTileManager *exttile = new ExtractorTileManager();
    for (uint i = 0; i < extractors.size(); ++i) {
        if (dispatchtable[i].first)
            exttile->Add(std::move(paths[dispatchtable[i].second]->GetPathTile()));
        else
            exttile->Add(std::move(films[dispatchtable[i].second]->GetFilmTile(tileBounds)));
    }

    return std::unique_ptr<ExtractorTileManager>(exttile);
}

void ExtractorManager::MergeTiles(std::unique_ptr<ExtractorTileManager> tiles) {
    for (uint i = 0; i < extractors.size(); ++i) {
        if (dispatchtable[i].first)
            paths[dispatchtable[i].second]->MergePathTile(std::move(tiles->GetPathTile(i)));
        else
            films[dispatchtable[i].second]->MergeFilmTile(std::move(tiles->GetFilmTile(i)));
    }
}

void ExtractorManager::WriteOutput(Float splatScale) {
    // TODO: Generic output
    for (uint i = 0; i < extractors.size(); ++i) {
        if (dispatchtable[i].first)
            paths[dispatchtable[i].second]->WriteFile();
        else
            films[dispatchtable[i].second]->WriteImage(splatScale);
    }
}

void ExtractorManager::AddSplats(const Point2f &pSplat, const Containers &containers) {
    for (uint i = 0; i < extractors.size(); ++i) {
        if (!dispatchtable[i].first)
            containers.AddSplats(i, pSplat, films[dispatchtable[i].second]);
    }
}

void ExtractorTileManager::AddSamples(const Point2f &pFilm,
                                      std::unique_ptr<Containers> containers, Float sampleWeight) {
    for (uint i = 0; i < dispatchtable.size(); ++i) {
        if (dispatchtable[i].first)
            pathtiles[dispatchtable[i].second]->AddSample(pFilm, containers->GetContainer(i));
        else
            filmtiles[dispatchtable[i].second]->AddSample(pFilm, containers->ToSample(i), sampleWeight);
    }
}


#endif

}