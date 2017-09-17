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
#if 0
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

    class PathExtractorContainer : public Container {
    public:
        PathExtractorContainer(const Point2f &pFilm, const std::regex &r, const std::string &regexpr) :
                pFilm(pFilm),
                regex(r),
                regexpr(regexpr),
                s_state(0),
                t_state(0) {};

        void Init(const RayDifferential &r, int depth, const Scene &Scene);

        void ReportData(const SurfaceInteraction &isect);

        void ReportData(const std::tuple<Spectrum, Float, Float, BxDFType> &bsdf);

        void ReportData(const Spectrum &L);

        Spectrum ToSample() const;

        std::vector<path_entry> GetPaths();

        void AddSplat(const Point2f &pSplat, Film *film);

        // TODO: If possible, check path regex before adding vertices to the container
        void BuildPath(const Vertex *lightVertices, const Vertex *cameraVertices, int s, int t);

    private:
        MemoryArena arena;
        const Point2f pFilm;
        const std::string regexpr;
        const std::regex regex;
        bool path_integrator = false;
        Interaction i; // Temporary storage for interaction collection
        // path state
        int s_state, t_state;
        Path current_path;
        std::map<std::pair<int, int>, Path> paths;
    };


    class PathExtractor : public ExtractorFunc {
    public:
        PathExtractor(const std::string &pathExpression) :
                r(std::regex(pathExpression, std::regex::optimize)), expr(pathExpression) {};

        std::shared_ptr<Container> GetNewContainer(const Point2f &p) const {
            return std::shared_ptr<Container>(new PathExtractorContainer(p, r, expr));
        }

        std::unique_ptr<PathExtractorContainer> GetNewPathExtractorContainer(const Point2f &p) const {
            return std::unique_ptr<PathExtractorContainer>(new PathExtractorContainer(p, r, expr));
        }

        ExtractorType GetType() const { return ExtractorType::PATH_EXTRACTOR; }
    private:
        const std::regex r;
        const std::string expr;
    };


    Extractor *CreatePathExtractor(const ParamSet &params, const Point2i &fullResolution,
                                   const Float diagonal, const std::string &imageFilename);
#endif
}

#endif //PBRT_EXTRACTOR_PATH_H
