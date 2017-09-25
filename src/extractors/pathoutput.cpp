//
// Created by stardami on 5/16/17.
//

#include <fstream>
#include <iomanip>
#include "pbrt.h"
#include "paramset.h"
#include "pathoutput.h"
#include "extractor.h"
#include "fileutil.h"

namespace pbrt {

void PathOutputTile::AddPath(const Point2f &pFilm, const path_entry &p) {
  tilepaths.push_back(p);
}


std::unique_ptr<PathOutputTile> PathOutput::GetPathTile() {
  return std::unique_ptr<PathOutputTile>(new PathOutputTile());
}

void PathOutput::MergePathTile(std::unique_ptr<PathOutputTile> tile) {
  ProfilePhase _(Prof::PathMergeTile);
  std::lock_guard<std::mutex> lock(mutex);

  // Path addition during rendering disabled (currently: slowing down rendering in text mode due to formatting)
  AppendPaths(tile->tilepaths);
}

void PathOutput::AppendPaths(const std::vector<path_entry> &entries) {
  ProfilePhase _(Prof::PathMergeTile);
  for(const path_entry &entry: entries) {
    if(HasExtension(filename, ".txtdump")) {
      f << "Path:";
      std::ostringstream str;
      str << entry;
      f << str.str() << "\n";
    } else {
      f << entry;
    }
  }
  npaths += entries.size();
}

void PathOutput::WriteFile() {
  ProfilePhase p(Prof::PathWriteOutput);
  // Seek to beginning and write header
  f.seekp(std::ios::beg);
  f << "Path file; n = " << npaths;
  f.close();
}


}