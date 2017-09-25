//
//

#ifndef PBRT_EXTRACTOR_PATH_H
#define PBRT_EXTRACTOR_PATH_H

#include "extractors/pathoutput.h"
#include <regex>
#include "pbrt.h"
#include "extractors/extractor.h"
#include "extractors/pathio.h"

namespace pbrt {

    enum class VertexInteraction {
        Camera, Light, Diffuse, Specular, Undef
    };
    static const char VertexNames[] = "ELDSU";

    inline uint64_t VertexInteractionToBits(VertexInteraction i) { return 1ull << (int) i; }

    struct PathVertex {
        // TODO: Constructor methods

        VertexInteraction type;
        Point3f p;
        Normal3f n;
        Spectrum f;    // BSDF spectrum
        Float pdf;
        Float pdf_rev;

        /*
        union {
            SurfaceInteraction si;
            Interaction ei; // TODO: Full EndpointInteraction support
            MediumInteraction mi; // TODO: handle medium interactions vertices
        };
        */

        PathVertex(const Point3f &p, VertexInteraction type = VertexInteraction::Undef) :
                p(p), type(type) {}

        PathVertex(const Interaction &isect, Float pdf, Float pdfRev, Spectrum f,
                   VertexInteraction type = VertexInteraction::Undef) :
                pdf(pdf), pdf_rev(pdfRev), p(isect.p), n(isect.n), f(f), type(type) {}

        static inline PathVertex FromBDPTVertex(const Vertex &v);

        friend std::ostream &operator<<(std::ostream &os, const PathVertex &v) {
            return os << v.ToString();
        }

        std::string ToString() const {
            std::string s = std::string("[v type: ");
            switch (type) {
                case VertexInteraction::Camera:
                    s += "camera";
                    break;
                case VertexInteraction::Light:
                    s += "light";
                    break;
                case VertexInteraction::Diffuse:
                    s += "diffuse interaction";
                    break;
                case VertexInteraction::Specular:
                    s += "specular interaction";
                    break;
                default:
                    s += "undefined interaction";
            }
            s += StringPrintf("; p: [ %f, %f, %f ]", p.x, p.y, p.z);
            s += std::string(" ]");
            return s;
        }
    };

    struct Path {
        Path(int length) : L(Spectrum(0.f)) {
            vertices.reserve(length);
        }

        Path() {};

        // Point2f pOrigin;
        Spectrum L;
        std::vector<PathVertex> vertices;

        std::string GetPathExpression() const {
            std::string s;
            for (const PathVertex &v : vertices) {
                s += VertexNames[(int) (v.type)];
            }

            return s;
        }

        bool isValidPath(const std::regex &pathPattern) const;

        friend std::ostream &operator<<(std::ostream &os, const Path &p) {
            return os << p.ToString();
        }

        std::string ToString() const {
            std::string s = std::string("[Path expr: \"");
            s += GetPathExpression() + "\" ;";
            s += " vertices --> ";

            for (int i = 0; i < vertices.size(); ++i) {
                s += StringPrintf(" p%d: [ %f, %f, %f ] ", i, vertices[i].p.x, vertices[i].p.y, vertices[i].p.z);
            }
            s += std::string(" ] ");

            s += " Luminance --> ";
            s += L.ToString();
            return s;
        }

    };


class ExtractorPath : public Extractor {
public:
    ExtractorPath(const std::string &pathExpression, std::unique_ptr<Film> f, std::unique_ptr<PathOutput> p) :
            Extractor(PATH_EXTRACTOR), regex(std::regex(pathExpression, std::regex::optimize)), regexpr(pathExpression), film(std::move(f)), path_file(std::move(p))
            {}

    ~ExtractorPath() {}

    // Tiling stuff
    std::unique_ptr<Extractor> BeginTile(const Bounds2i &tileBound) const;
    // End tile get a tiledExtractor as parameter and merge it with the current full extractor.
    // Tiled extractors might use this method with a null parameter to finalize the tile
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor = nullptr) const;

    // General stuff
    // What are the parameters of this ???
    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale = 1.f);


private:

    const std::regex regex;
    const std::string regexpr;

    std::unique_ptr<Film> film;
    std::unique_ptr<PathOutput> path_file;
};


class ExtractorPathTile : public Extractor {
public:
    ExtractorPathTile(Film *f, PathOutput *p, const Bounds2i &tileBound, const std::regex &reg, const std::string &exp);

    ~ExtractorPathTile() {}

    // Tiling stuff
    std::unique_ptr<Extractor> BeginTile(const Bounds2i &tileBound) const;
    void EndTile(std::unique_ptr<Extractor> sourceTiledExtractor = nullptr) const;

    // Pixel stuff
    void BeginPixel(const Point2i &pix);
    void EndPixel();

    // Sample stuff
    void BeginSample(const Point2f &p);
    void EndSample(const Spectrum &throughput, float weight = 1.f);

    // todo : do we need UpdatePixel ? which profile for such a method ?

    // Path (or Sample) stuff. May be simplified ?
    virtual void BeginPath(const Point2f &p);

    // incremental building of a path, usefull for SamplerIntegrators
    //virtual void InitializePath(const RayDifferential &r, int depth, const Scene &scene) {}
    /* TODO, see how to generate complete path vertices (like bdpt one) for path-tracing like integrators */
    void AddCameraVertex(Point3f o /* TODO : is this suffcient ? */);
    void AddLightVertex(/* TODO : find how to generate this kind of path vertex from a sampler integrator */);
    void AddPathVertex(const SurfaceInteraction &isect, const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf);
    // direct building of a path, usefull for bidirectionnal integrators
    void AddPathVertices(const Vertex *lightVertrices, const Vertex *cameraVertrices, int s, int t);
    void EndPath(const Spectrum &throughput, float weight = 1.f);

    // General stuff
    // What are the parameters of this ???
    void Initialize(const Bounds3f & worldBound);
    void Flush(float splatScale = 1.f);

    std::unique_ptr<FilmTile> GetFilmTile() {
        return std::move(film);
    }

    std::unique_ptr<PathOutputTile> GetPathsTile() {
        return std::move(paths);
    }

private:

    const std::regex regex;
    const std::string regexpr;

    Point2i pixel;
    MemoryArena arena;
    Point2f sample_pos;


    bool path_started;

    bool path_integrator;
    Interaction i; // Temporary storage for interaction collection

    // path state
    int s_state, t_state;
    Path current_path;

    Film *fullfilm;
    std::unique_ptr<FilmTile> film;
    std::unique_ptr<PathOutputTile> paths;
};


Extractor *CreatePathExtractor(const ParamSet &params, std::shared_ptr<const Camera> camera);


}

#endif //PBRT_EXTRACTOR_PATH_H
