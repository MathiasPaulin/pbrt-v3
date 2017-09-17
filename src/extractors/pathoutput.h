//
//

#ifndef PBRT_EXTRACTOR_PATHOUTPUT_H
#define PBRT_EXTRACTOR_PATHOUTPUT_H

#include <fstream>
#include <sstream>
#include "pbrt.h"
#include "core/geometry.h"
#include "core/parallel.h"
#include "core/memory.h"
#include "extractors/pathio.h"
#include <iomanip>

namespace pbrt {

#if 0
class PathOutput {
public:
    PathOutput(const std::string &filename) : filename(filename), f(filename, std::ios::binary), npaths(0) {
        // Reserve space for header
        const int headersize = 23 + 15; // 2^64 ~= 1e19 + 15 characters for the header
        f << std::left << std::setw(headersize) << "Path file; n =" << " " << std::endl;
    }

    std::unique_ptr<PathOutputTile> GetPathTile();

    void MergePathTile(std::unique_ptr<PathOutputTile> tile);

    void WriteFile();

private:
    void AppendPaths(const std::vector<path_entry> &entries);

    std::mutex mutex;
    std::vector<path_entry> paths;
    const std::string filename;
    std::ofstream f;
    uint64_t npaths;
};

class PathOutputTile {
public:
    void AddSample(const Point2f &pFilm, std::shared_ptr<Container> container);

private:
    MemoryArena arena; // Path storage arena
    // Path entries
    const Bounds2i pixelBounds;
    std::vector<path_entry> tilepaths;

    friend class PathOutput;
};

PathOutput *CreatePathOutput(const ParamSet &params);
#endif

} // namespace pbrt

#endif //PBRT_EXTRACTOR_PATHOUTPUT_H
