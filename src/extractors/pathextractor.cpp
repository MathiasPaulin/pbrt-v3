//
// Created by stardami on 5/3/17.
//

#include "extractors/pathextractor.h"
#include "extractors/pathio.h"
#include "integrators/bdpt.h"
#include "pbrt.h"
#include "paramset.h"
#include "filters/box.h"
#include <regex>
#include <algorithm>

namespace pbrt {



bool Path::isValidPath(const std::regex &pathPattern) const {
    ProfilePhase p(Prof::PathExtractorRegexTest);
    const std::string pathExpr = GetPathExpression();
    std::smatch base_match;

    return std::regex_match(pathExpr, base_match, pathPattern);
}

PathVertex PathVertex::FromBDPTVertex(const Vertex &v) {
    switch (v.type) {
        case VertexType::Light:
            return PathVertex(v.ei, v.pdfFwd, v.pdfRev, v.bsdf_f, VertexInteraction::Light);
        case VertexType::Camera:
            return PathVertex(v.ei, v.pdfFwd, v.pdfRev, Spectrum(0.f), VertexInteraction::Camera);
        case VertexType::Surface:
            return PathVertex(v.si, v.pdfFwd, v.pdfRev, v.bsdf_f,
                              v.delta ? VertexInteraction::Specular : VertexInteraction::Diffuse);
        default:
            LOG(FATAL) << "BDPT Vertex type not supported (Medium interaction) ?";
    }
}


// ExtractorPath
std::unique_ptr<Extractor> ExtractorPath::BeginTile(const Bounds2i &tileBound) const{
    return std::unique_ptr<Extractor>(new ExtractorPathTile(film.get(), path_file.get(), tileBound, regex, regexpr));
}

void ExtractorPath::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    ExtractorPathTile *source = static_cast<ExtractorPathTile *>(sourceTiledExtractor.get());
    if (film)
        film->MergeFilmTile(source->GetFilmTile());

    if (path_file)
        path_file->MergePathTile(source->GetPathsTile());
}

void ExtractorPath::Initialize(const Bounds3f & worldBound){
}

void ExtractorPath::Flush(float splatScale){
    if (film)
        // here, write the film to the file
        film->WriteImage(splatScale);
    if (path_file)
        path_file->WriteFile();
}


// ExtractorPathTile
ExtractorPathTile::ExtractorPathTile(Film *f, PathOutput *p, const Bounds2i &tileBound, const std::regex &reg, const std::string &exp) :
    Extractor(PATH_EXTRACTOR), regex(reg), regexpr(exp) {
    if (f) {
        fullfilm = f;
        film = f->GetFilmTile(tileBound);
    } else {
        fullfilm = nullptr;
    }

    paths = p->GetPathTile();
    path_integrator = false;
    path_started = false;
}

// Tiling stuff
std::unique_ptr<Extractor> ExtractorPathTile::BeginTile(const Bounds2i &tileBound) const{
    return std::unique_ptr<Extractor>(nullptr);
}

void ExtractorPathTile::EndTile(std::unique_ptr<Extractor> sourceTiledExtractor) const {
    // Nothing to do here
}

// Pixel stuff
void ExtractorPathTile::BeginPixel(const Point2i &pix){
    pixel=pix;
}

void ExtractorPathTile::EndPixel(){

}



void ExtractorPathTile::BeginSample(const Point2f &p) {
    sample_pos = p;
}

void ExtractorPathTile::EndSample(const Spectrum &throughput, float weight) {
    //sample_luminance = throughput * weight;
    // TODO ????
    // Must structure the pats into sample
    // a sample is a luminance resulting from the reconstruction of a set of paths
    //      A path is a set of vertices, a luminance and a weight
}

// Path (or Sample) stuff. May be simplified ?
void ExtractorPathTile::BeginPath(const Point2f &p){
    sample_pos = p;
    path_integrator = false;
    path_started = true;
}


void ExtractorPathTile::AddCameraVertex(Point3f o) {
    if (path_started) {
        path_integrator = true;
        i = Interaction();
        current_path = Path();
        // Eye vertex setup
        PathVertex eye(o, VertexInteraction::Camera);
        current_path.vertices.push_back(eye);
        t_state = 0;
        s_state = 0;
    }
}

void ExtractorPathTile::AddLightVertex() {
    // TODO ????
}


void ExtractorPathTile::AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf){
    // this function is to be used only with path tracing like method
    if (path_started && path_integrator) {
        i = isect;
        ++t_state;
        const VertexInteraction type =
                (std::get<3>(bsdf) & BSDF_SPECULAR) != 0 ? VertexInteraction::Specular : VertexInteraction::Diffuse;
        const Float pdf = std::get<1>(bsdf);
        const Float pdf_rev = std::get<2>(bsdf);
        const Spectrum bsdf_f = std::get<0>(bsdf);
        PathVertex v(i, pdf, pdf_rev, bsdf_f, type);
        current_path.vertices.push_back(v);
    }
}

// direct building of a path, usefull for bidirectionnal integrators
void ExtractorPathTile::AddPathVertices(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t){
    // this function is to be used with bdpt style method
    if (path_started && !path_integrator) {
        ProfilePhase p(Prof::PathExtractorBuildPath);
        // Light->Camera order
        // Light to camera vertices
        current_path = Path(s + t);

        // special case of s==0

        std::for_each(lightVertrices, lightVertrices + s, [&](const Vertex &v) {
            current_path.vertices.push_back(PathVertex::FromBDPTVertex(v));
        });

        // Camera to light vertices, must be added in reverse order
        /// special case for light-ending paths
        if (s == 0) {
            // Interpret the camera subpath as a complete path
            const Vertex &pt = cameraVertrices[t - 1];
            if (pt.IsLight())
                current_path.vertices.push_back(
                        PathVertex(pt.ei, pt.pdfFwd, pt.pdfRev, pt.bsdf_f, VertexInteraction::Light));
            else
                current_path.vertices.push_back(PathVertex::FromBDPTVertex(cameraVertrices[t - 1]));
        } else {
            current_path.vertices.push_back(PathVertex::FromBDPTVertex(cameraVertrices[t - 1]));
        }

        for (int i = t - 2; i >= 0; --i) {
            current_path.vertices.push_back(PathVertex::FromBDPTVertex(cameraVertrices[i]));
        }

        // Save last path state even if invalid
        s_state = s;
        t_state = t;
        // VLOG(2) << "Path built : " << current_path << "[ (s, t) --> (" << s_state << ", " << t_state << ") ]\n";
    }
}

void ExtractorPathTile::EndPath(const Spectrum &throughput, float weight) {
    // TODO group paths per pixel ?
    if (path_started) {
        if (!throughput.IsBlack()) {
            // report only paths with non zero radiance

            // Pseudo endpoint vertex
            if (path_integrator) {
                // Remove last vertex if no intersection occured
                if (current_path.vertices.back().type == VertexInteraction::Undef)
                    current_path.vertices.pop_back();

                // Reverse path for regex compatibility
                std::reverse(current_path.vertices.begin(), current_path.vertices.end());
            }

            current_path.L = throughput * weight;

            if (current_path.isValidPath(regex)) {
//                VLOG(2) << "Path added (matches)" << current_path << "[ (s, t) --> (" << s_state << ", " << t_state << ") ]\n";

                if (fullfilm != nullptr && !current_path.L.IsBlack()) {
                    if (path_integrator || t_state != 1) {
                        film->AddSample(sample_pos, current_path.L);
                    } else
                        fullfilm->AddSplat(sample_pos, current_path.L);
                }

                if (paths != nullptr) {
                    path_entry entry;
                    // Discard empty paths
                    entry.path = current_path.GetPathExpression();

                    if (entry.path.empty()) {
                        //                VLOG(1) << "Empty path not added" << entry.path << "\n";
                        return;
                    }

                    entry.regex = regexpr;
                    entry.regexlen = regexpr.size();
                    entry.pathlen = entry.path.size();
                    current_path.L.ToRGB(&entry.L[0]);
                    entry.pFilm[0] = sample_pos.x;
                    entry.pFilm[1] = sample_pos.y;
                    entry.vertices.reserve(current_path.vertices.size());

                    std::for_each(current_path.vertices.begin(), current_path.vertices.end(), [&](const PathVertex &v) {
                        vertex_entry vertex;
                        vertex.type = (uint32_t) v.type;
                        vertex.v = {v.p.x, v.p.y, v.p.z};
                        vertex.n = {v.n.x, v.n.y, v.n.z};
                        v.f.ToRGB(&vertex.bsdf[0]);
                        vertex.pdf_in = v.pdf_rev;
                        vertex.pdf_out = v.pdf;

                        entry.vertices.push_back(vertex);
                    });

                    paths->AddPath(sample_pos, entry);
                }
            }
        }
        current_path = Path(); // Clear previous path
        t_state = 0;
        s_state = 0;
        path_started = false;
    }
}

// General stuff
// What are the parameters of this ???
void ExtractorPathTile::Initialize(const Bounds3f & worldBound){

}

// Todo : perhaps splat here to paths that reside outside of the tile
void ExtractorPathTile::Flush(float splatScale){

}


// To be modified

Extractor *CreatePathExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera) {
    // if outputfile is different from "none", construct pathfilename from the value
    std::string filename = params.FindOneString("outputfile", "none");
    PathOutput *pathoutput = nullptr;
    if (filename != "none") {
        if (filename.empty()) filename = "paths_" + camera->film->filename;
        pathoutput = new PathOutput(filename);
    }

    // if imagefile is different from "none", construct imagefilename from the value
    std::string imagefile = params.FindOneString("imagefile", "none");
    Film *filmoutput = nullptr;
    if (imagefile != "none") {
        if (imagefile.empty()) imagefile = "pathsimage_" + camera->film->filename;
        filmoutput = new Film(camera->film->fullResolution,
                              Bounds2f(Point2f(0, 0), Point2f(1, 1)),
                              std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
                              camera->film->diagonal, imagefile, 1.f);
    }

    std::string regex = params.FindOneString("regex", "");
    return new ExtractorPath(regex, std::move(std::unique_ptr<Film>(filmoutput)), std::move(std::unique_ptr<PathOutput>(pathoutput)));
}

/*
 // Group path per type (sample or splat)and per pixels (for samples)
 // This might allow to reconstruct the picture in the same way than bdtp does.
Spectrum PathExtractorContainer::ToSample() const {
    Spectrum L(0.f);
    for (const auto &path : paths) {
        VLOG(1) << "Accumulating path luminance " << path.second << "\n";
        L += path.second.L;
    }
    return L;
}

void PathExtractorContainer::AddSplat(const Point2f &pSplat, Film *film) {
    // TODO: group w/ ToSample
    auto path = paths.find({s_state, t_state});
    if (path != paths.end()) {
        film->AddSplat(pSplat, path->second.L);
        paths.erase(path); // Remove splatted path contribution from sampled paths vector
    }
}

*/

}