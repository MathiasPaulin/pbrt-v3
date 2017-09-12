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

void PathExtractorContainer::Init(const RayDifferential &r, int depth, const Scene &Scene) {
    // Path tracing bounce init
    if (depth == 0) {
        i = Interaction(); // clear last interaction state
        path_integrator = true;
        // Path setup
        current_path = Path();

        // Eye vertex setup
        PathVertex eye(r.o, VertexInteraction::Camera);
        current_path.vertices.push_back(eye);
    } else {
        // New potential bounce

    }
}

// Path Tracing extraction methods
void PathExtractorContainer::ReportData(const SurfaceInteraction &isect) {
    // New surface interaction = confirmed bounce
    i = isect;
}

void PathExtractorContainer::ReportData(const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf) {
    const VertexInteraction type =
            (std::get<3>(bsdf) & BSDF_SPECULAR) != 0 ? VertexInteraction::Specular : VertexInteraction::Diffuse;
    const Float pdf = std::get<1>(bsdf);
    const Float pdf_rev = std::get<2>(bsdf);
    const Spectrum bsdf_f = std::get<0>(bsdf);
    PathVertex v(i, pdf, pdf_rev, bsdf_f, type);
    current_path.vertices.push_back(v);
}

// BDPT extraction methods
void PathExtractorContainer::BuildPath(const Vertex *lightVertices, const Vertex *cameraVertices, int s, int t) {
    ProfilePhase p(Prof::PathExtractorBuildPath);
    // Light->Camera order
    // Light to camera vertices
    current_path = Path(s + t);

    std::for_each(lightVertices, lightVertices + s, [&](const Vertex &v) {
        current_path.vertices.push_back(PathVertex::FromBDPTVertex(v));
    });

    // Camera to light vertices, must be added in reverse order
    for (int i = t - 1; i >= 0; --i) {
        current_path.vertices.push_back(PathVertex::FromBDPTVertex(cameraVertices[i]));
    }

    // Save last path state even if invalid
    s_state = s;
    t_state = t;
}

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

void PathExtractorContainer::ReportData(const Spectrum &L) {
    // Pseudo endpoint vertex
    if (path_integrator) {
        // Remove last vertex if no intersection occured
        if (current_path.vertices.back().type == VertexInteraction::Undef)
            current_path.vertices.pop_back();

        // Reverse path for regex compatibility
        std::reverse(current_path.vertices.begin(), current_path.vertices.end());
    }

    current_path.L = L;
    if (current_path.isValidPath(regex)) {
        VLOG(1)
        << "New path added (matches)" << current_path << "[ (s, t) --> (" << s_state << ", " << t_state << ") ]\n";
        paths[{s_state, t_state}] = current_path;
    }

    current_path = Path(); // Clear previous path
}


std::vector<path_entry> PathExtractorContainer::GetPaths() {
    ProfilePhase p(Prof::PathExtractorToPathSample);
    std::vector<path_entry> entries;

    std::for_each(paths.cbegin(), paths.cend(),
                  [&](const std::pair<std::pair<int, int>, Path> &kv) {
                      const Path &p = kv.second;
                      path_entry entry;

                      // Discard empty paths
                      entry.path = p.GetPathExpression();

                      if (entry.path.empty()) {
                          VLOG(2) << "Empty path not added" << entry.path << "\n";
                          return;
                      }

                      entry.regex = regexpr;
                      entry.regexlen = regexpr.size();
                      entry.pathlen = entry.path.size();
                      p.L.ToRGB(&entry.L[0]);
                      entry.pFilm[0] = pFilm.x;
                      entry.pFilm[1] = pFilm.y;
                      entry.vertices.reserve(p.vertices.size());

                      std::for_each(p.vertices.begin(), p.vertices.end(), [&](const PathVertex &v) {
                          vertex_entry vertex;
                          vertex.type = (uint32_t) v.type;
                          vertex.v = {v.p.x, v.p.y, v.p.z};
                          vertex.n = {v.n.x, v.n.y, v.n.z};
                          v.f.ToRGB(&vertex.bsdf[0]);
                          vertex.pdf_in = v.pdf_rev;
                          vertex.pdf_out = v.pdf;

                          entry.vertices.push_back(vertex);
                      });

                      entries.push_back(entry);
                  });

    return entries;
}


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

Extractor *CreatePathExtractor(const ParamSet &params, const Point2i &fullResolution,
                               Float diagonal, const std::string &imageFilename) {
    std::string filename = params.FindOneString("outputfile", "");
    if (filename.empty()) filename = "pextract_" + imageFilename;

    std::string regex = params.FindOneString("regex", "");

    if (HasExtension(filename, ".txtdump") || HasExtension(filename, ".bindump")) {
        return new Extractor(new PathExtractor(regex), new PathOutput(filename));
    } else {
        return new Extractor(new PathExtractor(regex), new Film(
                fullResolution,
                Bounds2f(Point2f(0, 0), Point2f(1, 1)),
                std::unique_ptr<pbrt::Filter>(new BoxFilter(Vector2f(1.f, 1.f))),
                35.f, filename, 1.f));
    }
}

}