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


std::unique_ptr<Extractor> ExtractorSet::BeginTile(const Bounds2i &tileBound) const {
    ProfilePhase p(Prof::ExtractorReport);
    ExtractorSet *tileSet = new ExtractorSet;
    for (const auto &e : extractors)
        tileSet->AddExtractor(std::move(e.second->BeginTile(tileBound)));
    return std::unique_ptr<Extractor>(tileSet);
}

void ExtractorSet::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    ProfilePhase p(Prof::ExtractorReport);
    // merge tile (source in e) in the current extractor set (target)
    CHECK( sourceTiledExtractor->Type() == EXTRACTOR_SET );
    auto source = static_cast<ExtractorSet * const>(sourceTiledExtractor.get());
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
void ExtractorSet::BeginPixel(const Point2i &pix) {
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->BeginPixel(pix);
}

void ExtractorSet::EndPixel()  {
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->EndPixel();
}


void ExtractorSet::BeginSample(const Point2f &p) {
    ProfilePhase pp(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->BeginSample(p);
}

void ExtractorSet::EndSample(const Spectrum &throughput, float weight) {
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->EndSample(throughput, weight);
}

// Path (or Sample) stuff. May be simplified ?
void ExtractorSet::BeginPath(const Point2f &p)  {
    ProfilePhase pp(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->BeginPath(p);
}

void ExtractorSet::AddCameraVertex(Point3f o) {
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->AddCameraVertex(o);
}

void ExtractorSet::AddLightVertex() {
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->AddLightVertex();
}

void ExtractorSet::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->AddPathVertex(isect, bsdf);
}

// direct building of a path, usefull for bidirectionnal integrators
void ExtractorSet::AddPathVertices(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t) {
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->AddPathVertices(lightVertrices, cameraVertrices, s, t);
}

void ExtractorSet::EndPath(const Spectrum &throughput, float weight){
    ProfilePhase p(Prof::ExtractorReport);
    for (const auto &e : extractors)
        e.second->EndPath(throughput, weight);
}

void ExtractorSet::Initialize(const Bounds3f &worldBound) {
    ProfilePhase p(Prof::ExtractorInit);
    for (const auto &e : extractors)
        e.second->Initialize(worldBound);
}

void ExtractorSet::Flush(float splatScale) {
    ProfilePhase p(Prof::ExtractorWriteOuput);
    for (const auto &e : extractors)
        e.second->Flush(splatScale);
}


std::unique_ptr<Extractor> ExtractorNormal::BeginTile(const Bounds2i &tileBound) const {
    // generate an ExtractorNormalTile, it and return it
    return std::unique_ptr<Extractor>(new ExtractorNormalTile(film.get(),tileBound));
}

void ExtractorNormal::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    // Verify that e is an ExtractornormalTile and merge it with current film
    // well, for the moment, we assume it is the case (due to properties of multimap)
    ExtractorNormalTile *source = static_cast<ExtractorNormalTile *>(sourceTiledExtractor.get());
    film->MergeFilmTile(source->GetTile());
}

// General stuff
// What are the parameters of this ???
void ExtractorNormal::Initialize(const Bounds3f &worldBound){
}

void ExtractorNormal::Flush(float splatScale){
    // here, write the film to the file
    film->WriteImage(splatScale);
}

ExtractorNormalTile::ExtractorNormalTile(Film *f, const Bounds2i &tileBound) : Extractor(FILM_EXTRACTOR){
    film =f->GetFilmTile(tileBound);
}

std::unique_ptr<Extractor> ExtractorNormalTile::BeginTile(const Bounds2i &tileBound) const{
    return std::unique_ptr<Extractor>(nullptr);
}

void ExtractorNormalTile::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    // Nothing to do here
}

// Path (or Sample) stuff. May be simplified ?
void ExtractorNormalTile::BeginPath(const Point2f &p) {
    samplePos = p;
    valid_path = false;
    first = true;
}

void ExtractorNormalTile::AddCameraVertex(Point3f o) {
    first = true;
    valid_path = false;
}

void ExtractorNormalTile::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    if (first) {
        n = Faceforward(isect.n, isect.wo);
        first = false;
        valid_path = true;
    }
}

void ExtractorNormalTile::AddPathVertices(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t) {
    // if t==1 BDPT use addSplat on the entire film. find a way to manage normal extractors for bdpt ...
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

void ExtractorNormalTile::EndPath(const Spectrum &throughput, float weight) {
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
void ExtractorNormalTile::Initialize(const Bounds3f & worldBound) {
}
void ExtractorNormalTile::Flush(float splatScale) {
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

ExtractorDepth::ExtractorDepth(Film *f, float zn, float zf) : Extractor(FILM_EXTRACTOR), film(f), znear(zn), zfar(zf) {

}

std::unique_ptr<Extractor> ExtractorDepth::BeginTile(const Bounds2i &tileBound) const {
    // generate an ExtractorNormalTile, it and return it
    return std::unique_ptr<Extractor>(new ExtractorDepthTile(film.get(),tileBound, znear, zfar));
}

void ExtractorDepth::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    // Verify that e is an ExtractornormalTile and merge it with current film
    // well, for the moment, we assume it is the case (due to propertiez of multimap)
    ExtractorDepthTile *source = static_cast<ExtractorDepthTile *>(sourceTiledExtractor.get());
    film->MergeFilmTile(source->GetTile());
}

// General stuff
// What are the parameters of this ???
void ExtractorDepth::Initialize(const Bounds3f & worldBound){
}

void ExtractorDepth::Flush(float splatScale){
    // here, write the film to the file
    film->WriteImage(splatScale);
}



ExtractorDepthTile::ExtractorDepthTile(Film *f, const Bounds2i &tileBound, float zn, float zf) :
        Extractor(FILM_EXTRACTOR),
        znear(zn), zfar(zf),
        realdepth(0.f) {
    film =f->GetFilmTile(tileBound);
    zscale = ( (zfar <= znear) ? 1.f : (zfar / (zfar - znear)) );
}

std::unique_ptr<Extractor> ExtractorDepthTile::BeginTile(const Bounds2i &tileBound) const{
    return std::unique_ptr<Extractor>(nullptr);
}

void ExtractorDepthTile::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    // Nothing to do here
}

void ExtractorDepthTile::BeginPath(const Point2f &p) {
    samplePos = p;
    valid_path = false;
}

void ExtractorDepthTile::AddCameraVertex(Point3f o) {
    first = true;
    rayorigin = o;
    valid_path = false;
}

void ExtractorDepthTile::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    if ( first ) {
        float z = Vector3f(isect.p - rayorigin).Length();
        realdepth = ConvertDepth(z);
        first = false;
        valid_path = true;
    }
}

void ExtractorDepthTile::AddPathVertices(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t){
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

void ExtractorDepthTile::EndPath(const Spectrum &throughput, float weight){
    (void)throughput;
    (void)weight;
    if (valid_path) {
        film->AddSample(samplePos, Spectrum(realdepth), 1.0);
        valid_path = false;
    }
}

void ExtractorDepthTile::Initialize(const Bounds3f &worldBound) {
}

void ExtractorDepthTile::Flush(float splatScale) {
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



ExtractorAlbedo::ExtractorAlbedo(Film *f, BxDFType bxdfType, bool integrateAlbedo, int nbSamples) :
        Extractor(FILM_EXTRACTOR), film(f),
        bxdf_type(bxdfType), integrate_albedo(integrateAlbedo), nb_samples(nbSamples)
{

}

std::unique_ptr<Extractor> ExtractorAlbedo::BeginTile(const Bounds2i &tileBound) const {
    // generate an ExtractorNormalTile, it and return it
    return std::unique_ptr<Extractor>(new ExtractorAlbedoTile(film.get(),tileBound, bxdf_type, integrate_albedo, nb_samples));
}

void ExtractorAlbedo::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    // Verify that e is an ExtractornormalTile and merge it with current film
    // well, for the moment, we assume it is the case (due to propertiez of multimap)
    ExtractorAlbedoTile *source = static_cast<ExtractorAlbedoTile *>(sourceTiledExtractor.get());
    film->MergeFilmTile(source->GetTile());
}

void ExtractorAlbedo::Initialize(const Bounds3f &worldBound){
}

void ExtractorAlbedo::Flush(float splatScale){
    // here, write the film to the file
    film->WriteImage(splatScale);
}

ExtractorAlbedoTile::ExtractorAlbedoTile(Film *f, const Bounds2i &tileBound, BxDFType bxdfType, bool integrateAlbedo, int nbSamples) :
        Extractor(FILM_EXTRACTOR),
        bxdf_type(bxdfType), integrate_albedo(integrateAlbedo), nb_samples(nbSamples)
{
    film =f->GetFilmTile(tileBound);
    if (integrate_albedo) {
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

std::unique_ptr<Extractor> ExtractorAlbedoTile::BeginTile(const Bounds2i &tileBound) const {
    return std::unique_ptr<Extractor>(nullptr);
}

void ExtractorAlbedoTile::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    // Nothing to do here
}

void ExtractorAlbedoTile::BeginPath(const Point2f &p) {
    samplePos = p;
    valid_path = false;
}

void ExtractorAlbedoTile::AddCameraVertex(Point3f o) {
    first = true;
    valid_path = false;
}

Spectrum ExtractorAlbedoTile::computeAlbedo(BSDF *bsdf, const Vector3f &dir){
    Point2f dummy;
    return integrate_albedo ?
              bsdf->rho(nb_samples, wi.data(), wo.data(), bxdf_type) :
              bsdf->rho(dir, 0, &dummy, BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE));
}

void ExtractorAlbedoTile::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    if ( first ) {
        if (isect.bsdf) {
            rho = computeAlbedo(isect.bsdf, isect.wo);
        }
        first = false;
        valid_path = true;
    }
}

void ExtractorAlbedoTile::AddPathVertices(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t){
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

void ExtractorAlbedoTile::EndPath(const Spectrum &throughput, float weight){
    (void)throughput;

    if (valid_path)
        film->AddSample(samplePos, Spectrum(rho), weight);
    valid_path = false;
}

void ExtractorAlbedoTile::Initialize(const Bounds3f &worldBound) {
}

void ExtractorAlbedoTile::Flush(float splatScale) {
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

}