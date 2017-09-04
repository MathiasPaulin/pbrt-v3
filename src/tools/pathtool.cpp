//
// Path file manipulation tool
//
// Error/Usage fct from imgtool.cpp

#include "extractors/pathio.h"
#include <cstring>
#include <regex>
#include <fstream>
#include "tools/pathtool.h"
#include "tools/classification/src/kmgen.h"
#include "tools/classification/src/kmedoids.h"
#include <vector>
#include <cstdarg>
#include <filters/box.h>
#include "core/film.h"
#include "core/paramset.h"
#include <memory>
#include <filters/triangle.h>

void label_to_img(PathFile &paths, const std::string &filename, int xres, int yres, float diagonal, const std::vector<uint64_t> &elements);

namespace pbrt {


void bin_to_txt(int argc, char *argv[]);
void path_grep(int argc, char *argv[]);
void print_stats(int argc, char *argv[]);
void align_check(int argc, char *argv[]);
void mmap_test(int argc, char *argv[]);

static void usage(const char *msg = nullptr, ...) {
  if (msg) {
    va_list args;
    va_start(args, msg);
    fprintf(stderr, "pathtool: ");
    vfprintf(stderr, msg, args);
    fprintf(stderr, "\n");
  }
  fprintf(stderr, R"(usage: pathtool <command> [options] <filenames...>

commands: cat, aligncheck, spherefilter, regexfilter, lengthfilter

cat option:
    --outfile          Output file name

lengthfilter option:
    syntax: pathtool lengthfilter <length> <filename>

regexfilter option:
    syntax: pathtool regexfilter <regex> <filename>

spherefilter option:
    syntax: pathtool spherefilter <radius> <x> <y> <z> <filename>

)");
  exit(1);
}

void align_check(int argc, char *argv[]) {
  if(argc != 3) {
    usage("Error: no file provided");
  }

  FILE *fp = fopen(argv[2], "r");

  if(!fp) {
    perror("Input file error:");
    exit(EXIT_FAILURE);
  }

  // Read header
  char header[80];
  char *pathcountptr;
  if(!fgets(header, 79, fp) || !(pathcountptr = strstr(header, "Path file; n = "))) {
    perror("Error finding header");
    exit(EXIT_FAILURE);
  }

  int64_t pathcount = strtol(pathcountptr+15, nullptr, 10); // 15 = length of header string "Path file.."
  std::cout << "Header found, reported path count: " << pathcount << std::endl;

  char buf[10];
  int failcount = 0;
  // For now just check for path alignment
  for (int64_t i = 0; i < pathcount; ++i) {
    uint32_t reglen;
    uint32_t pathlen;
    fread(&reglen, 4, 1, fp);
    fread(&pathlen, 4, 1, fp);

    // Skip path length (regxp+path string bytes + pathlen vertex entries)
    long int offset = reglen + pathlen * (1 + sizeof(vertex_entry));

    fseek(fp, offset, SEEK_CUR);
    // TODO: path coherence check (normalized vectors, regexp/expr check, plausible path, pdf values..)

    if(!(i % (1024*512))){
      std::cout << "[" << (i/(float)pathcount)*100.f << "%] \t" << i << " paths checked out of " << pathcount << std::endl;
    }
  }

  if(!failcount) {
    std::cout << "Check finished, no alignment error found" << std::endl;
  } else {
    std::cout << "Check finished, " << failcount << " errors found." << std::endl;
  }

}

void bin_to_txt(int argc, char *argv[]) {
  if(argc == 2)
    usage("no file provided");

  // TODO: Vérifier intégrité du fichier avant conversion

  char *infile = nullptr;
  char *outfile = nullptr;
  bool aligncheck = false;

  // Get opts
  if(argc == 5 && !strcmp(argv[2], "--outfile")) {
    outfile = argv[3];
    infile = argv[4];
  } else if (argc == 4 && !strcmp(argv[2], "--tostdout")) {
    outfile = 0; // To stdout
    infile = argv[3];
  } else usage("Invalid argument");

  FILE *fi = nullptr;
  FILE *fo = nullptr;

  if(!(fi = fopen(infile, "r"))) {
    perror("Input fopen:");
    exit(EXIT_FAILURE);
  }

  if(!(fo = !outfile ? stdout : fopen(outfile, "w"))) {
    perror("Output file error:");
    exit(EXIT_FAILURE);
  }

  // Skip header and read file
  char header[80];
  char *pathcountptr;
  if(!fgets(header, 79, fi) || !(pathcountptr = strstr(header, "Path file; n = "))) {
    perror("Error finding header");
    exit(EXIT_FAILURE);
  }

  int64_t pathcount = strtol(pathcountptr, nullptr, 10);
  std::cout << "Header found, reported path count: " << pathcount << std::endl;

  std::cout << header << std::endl;
  int cpt = 0;
  char buf[10]; // tmpbuf
  while(!pathcount--) {
    path_entry path;
    fread(&path.regexlen, 4, 1, fi);
    fread(&path.pathlen, 4, 1, fi);
    path.regex.resize(path.regexlen);
    path.path.resize(path.pathlen);
    fread(&path.regex[0], 1, path.regexlen, fi);
    fread(&path.path[0], 1, path.pathlen, fi);
    path.vertices.resize(path.pathlen);
    fread(&path.vertices[0], sizeof(vertex_entry), path.pathlen, fi);

    std::ostringstream str;
    str << path;
    ++cpt;

    fwrite(str.str().c_str(), sizeof(char), str.str().size(), fo);
    fputc('\n', fo);
  }

  fclose(fi);
  fflush(fo);
  fclose(fo);
}

// PathFile version of cat
void bin_to_txt2(int argc, char *argv[]) {
  std::ofstream out(argv[2]);

  PathFile file(argv[3]);

  for(const pbrt::path_entry &p : file) {
    std::ostringstream str;
    std::cout << (str << p).str() << std::endl;
  }
}

} // namespace pbrt


// Path select functions


// Regex match
static bool regMatch(const pbrt::path_entry &path, const std::string &regexstr) {
  std::regex reg(regexstr);
  return !strcmp(regexstr.c_str(), path.regex.c_str()) | std::regex_match(path.path, reg);
}

// Vertices around a sphere of radius r
static bool sphereSearch(const pbrt::path_entry &path, float r, float pos[3]) {
  for (const pbrt::vertex_entry &v : path.vertices) {
    float sum = 0.f;
    for (int j = 0; j < 3; ++j) {
      sum += (v.v[j] - pos[j])*(v.v[j] - pos[j]);
    }

    if(sum < r*r)
      return true;
  }

  return false;
}


void mmap_test(int argc, char* argv[]) {
  PathFile pathfile(argv[2]);

  for (const pbrt::path_entry &path : pathfile) {
    std::cout << "Path length " << path.pathlen << std::endl;
  }

  std::cout << "Average path length: " << pathfile.average_length() << std::endl;
}

void filter_by_length(int argc, char* argv[]) {
  if(argc != 4) {
    pbrt::usage();
  }

  // TODO: file name checks
  int length = std::atoi(argv[2]);
  int n = 0;

  PathFile file(argv[3]);
  std::for_each(file.begin(), file.end(), [&](const pbrt::path_entry &p) { n += p.pathlen == length ? 1 : 0; });

  std::cout << "Total paths of length " << length << " : " << n << std::endl;
}

void filter_by_location(int argc, char* argv[]) {
  if(argc != 7) {
    pbrt::usage();
  }

  float radius = std::stof(argv[5]);
  float pos[3] = {std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4])};

  std::cout << "Matching paths passing sphere of center " << pos[0] << ", " << pos[1] << ", " << pos[2] << " and radius " << radius << std::endl;
  PathFile file((std::string(argv[6])));
  std::vector<pbrt::path_entry> resultpaths;
  std::copy_if(file.begin(), file.end(), std::back_inserter(resultpaths),
               [&](const pbrt::path_entry &p) { return sphereSearch(p, radius, pos); });

  std::cout << "Number of paths matching : " << resultpaths.size() << std::endl;
}

void filter_by_regex(int argc, char* argv[]) {
  if(argc != 5) {
    pbrt::usage();
  }

  std::string regex(argv[2]);

  PathFile file((std::string(argv[3])));
  std::vector<pbrt::path_entry> resultpaths;
  std::copy_if(file.begin(), file.end(), std::back_inserter(resultpaths),
               [&](const pbrt::path_entry &p) { return regMatch(p, regex); });

//  std::cout << "Number of paths matching : " << resultpaths.size() << std::endl;

  std::cerr << "Reconstrucing " << regex << " paths (" << resultpaths.size() <<") ... " << std::endl;

  std::string outputfile(argv[4]);
  pbrt::Film *film =  new pbrt::Film(pbrt::Point2i(1024,1024),
                                     pbrt::Bounds2f(pbrt::Point2f(0, 0), pbrt::Point2f(1, 1)),
                                     std::unique_ptr<pbrt::Filter>(new pbrt::TriangleFilter(pbrt::Vector2f(1.f, 1.f))), 35.f, outputfile, 1.f);

  std::unique_ptr<pbrt::FilmTile> tile(film->GetFilmTile(pbrt::Bounds2i(pbrt::Point2i(0,0), pbrt::Point2i(1024,1024))));


  for(const pbrt::path_entry &p : resultpaths) {
    if(p.pathlen == 0) continue;
    pbrt::Point2f samplepoint(p.pFilm[0], p.pFilm[1]);
    pbrt::Spectrum bsdf = pbrt::Spectrum::FromRGB(&p.L[0]);
    //std::cerr << "Adding path " << samplepoint << "with bsdf = " << bsdf << std::endl;
    tile->AddSample(samplepoint, bsdf);
  }

  film->MergeFilmTile(std::move(tile));
  film->WriteImage();
  std::cerr << " ... done!" << std::endl;
}


void distance_classification(int argc, char* argv[]) {
  if(argc < 3) {
    pbrt::usage("Missing arguments");
    return;
  }

  const int k = std::atoi(argv[2]);
  PathFile file(argv[3]);
  // TODO: CreateGenerator
  std::shared_ptr<Kmeans::CentroidGenerator> generator(new Kmeans::DistanceGenerator(file));
  Kmeans::Classifier classifier(k, file, generator, 100);

  classifier.run();

  std::vector<std::shared_ptr<Kmeans::Label>> labels = classifier.getLabels();

  std::cout << "Classification results:" << std::endl;
  for (std::shared_ptr<Kmeans::Label> label : labels) {
    std::cout << "New label. Size " << label->size() << std::endl;
  }
}

void leveinstein_classification(int argc, char* argv[]) {
  if(argc < 3) {
    pbrt::usage("Missing arguments");
    return;
  }

  const int samplesize = 10000;
  const int iterations = 15;
  const int k = std::atoi(argv[2]);
  PathFile file(argv[3]);
  // TODO: CreateGenerator
  std::shared_ptr<Kmedoids::CentroidGenerator> generator(new Kmedoids::LevenshteinGenerator(file));
  Kmedoids::Classifier classifier(k, file, generator, samplesize, iterations);

  classifier.run();

  std::vector<std::shared_ptr<Kmedoids::Label>> labels = classifier.getLabels();

  std::cout << "Classification results:" << std::endl;
  for (int i = 0; i < labels.size(); ++i) {
    std::cout << "New label. Size " << labels[i]->size() << std::endl;
    std::string fname = "labels_" + std::to_string(i) + ".exr";
    label_to_img(file, fname, 1024, 1024, .35f, labels[i]->elements);
  }

}


void label_to_img(PathFile &paths, const std::string &filename, int xres, int yres, float diagonal, const std::vector<uint64_t> &elements) {
  pbrt::Film *film =  new pbrt::Film(pbrt::Point2i(xres, yres),
                                     pbrt::Bounds2f(pbrt::Point2f(0, 0), pbrt::Point2f(1, 1)),
                                     std::unique_ptr<pbrt::Filter>(new pbrt::TriangleFilter(pbrt::Vector2f(1.f, 1.f))), diagonal, filename, 1.f);

  std::unique_ptr<pbrt::FilmTile> tile(film->GetFilmTile(pbrt::Bounds2i(pbrt::Point2i(0,0), pbrt::Point2i(xres,yres))));
  for(uint64_t idx : elements) {
    const pbrt::path_entry &p = paths[idx];
    if(p.pathlen == 0) continue;
    pbrt::Point2f samplepoint(p.pFilm[0], p.pFilm[1]);
    pbrt::Spectrum bsdf = pbrt::Spectrum::FromRGB(&p.L[0]);
    //std::cerr << "Adding path " << samplepoint << "with bsdf = " << bsdf << std::endl;
    tile->AddSample(samplepoint, bsdf);
  }

  std::cerr << "Merging path tile" << std::endl;

  film->MergeFilmTile(std::move(tile));
  film->WriteImage();
}

void pathdist_classification(int argc, char* argv[]) {
  if(argc < 3) {
    pbrt::usage("Missing arguments");
    return;
  }

  const int samplesize = 10000;
  const int iterations = 20;
  const int k = std::atoi(argv[2]);
  PathFile file(argv[3]);
  // TODO: CreateGenerator
  std::shared_ptr<Kmedoids::CentroidGenerator> generator(new Kmedoids::PathDistanceGenerator(file));
  Kmedoids::Classifier classifier(k, file, generator, samplesize, iterations);

  classifier.run();

  std::vector<std::shared_ptr<Kmedoids::Label>> labels = classifier.getLabels();

  std::cout << "Classification results:" << std::endl;
  for (int i = 0; i < labels.size(); ++i) {
    std::cout << "New label. Size " << labels[i]->size() << std::endl;
    std::string fname = "labels_" + std::to_string(i) + ".exr";
    label_to_img(file, fname, 1024, 1024, .35f, labels[i]->elements);
  }

}


void path_to_img(int argc, char* argv[]) {
  PathFile paths(argv[2]);
  std::string outputfile(argv[3]);

  pbrt::Film *film =  new pbrt::Film(pbrt::Point2i(1024,1024),
          pbrt::Bounds2f(pbrt::Point2f(0, 0), pbrt::Point2f(1, 1)),
          std::unique_ptr<pbrt::Filter>(new pbrt::TriangleFilter(pbrt::Vector2f(1.f, 1.f))), 35.f, outputfile, 1.f);

  std::unique_ptr<pbrt::FilmTile> tile(film->GetFilmTile(pbrt::Bounds2i(pbrt::Point2i(0,0), pbrt::Point2i(1024,1024))));
  for(const pbrt::path_entry &p : paths) {
    if(p.pathlen == 0) continue;
    pbrt::Point2f samplepoint(p.pFilm[0], p.pFilm[1]);
    pbrt::Spectrum bsdf = pbrt::Spectrum::FromRGB(&p.L[0]);
    //std::cerr << "Adding path " << samplepoint << "with bsdf = " << bsdf << std::endl;
    tile->AddSample(samplepoint, bsdf);
  }

  std::cerr << "Merging path tile" << std::endl;

  film->MergeFilmTile(std::move(tile));
  film->WriteImage();

}

int main(int argc, char* argv[]) {

  if(argc == 1) {
    pbrt::usage("");
    return 1;
  }

  if(!strcmp(argv[1], "cat")) {
    pbrt::bin_to_txt(argc, argv);
  } else if(!strcmp(argv[1], "aligncheck")) {
    pbrt::align_check(argc, argv);
  } else if (!strcmp(argv[1], "mmaptest")) {
    mmap_test(argc, argv);
  } else if (!strcmp(argv[1], "lengthfilter")) {
    filter_by_length(argc, argv);
  } else if (!strcmp(argv[1], "regexfilter")) {
    filter_by_regex(argc, argv);
  } else if (!strcmp(argv[1], "spherefilter")) {
    filter_by_location(argc, argv);
  } else if (!strcmp(argv[1], "cat2")) {
    pbrt::bin_to_txt2(argc, argv);
  } else if(!strcmp(argv[1], "distance")) {
    distance_classification(argc, argv);
  } else if(!strcmp(argv[1], "pathdist")) {
    pathdist_classification(argc, argv);
  } else if(!strcmp(argv[1], "lev")) {
    leveinstein_classification(argc, argv);
  } else if(!strcmp(argv[1], "toimg")) {
    path_to_img(argc, argv);
  } else {
    pbrt::usage("");
  }

  return 0;
}
