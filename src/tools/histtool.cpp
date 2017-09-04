//
// Path file histogram tool
//
// Error/Usage fct from imgtool.cpp

#include "tools/pathtool.h"
#include <fstream>
#include <iomanip>
#include <cstdarg>
#include <set>
#include <regex>
#include <numeric>
/**
 * Input values file structure
 *
 * [Field]
 * ..values selected for histogram sets..
 *
 * Exemple for a filtering by path length
 * [ELength]
 * 1 2 3 4 5 7 9
 *
 * Will generate an histogram showing paths of a given length + other paths
 *
 */

// Manipulation enum
enum EType {
    RMatch,
    RLength,
    ERegex,
    ELength,
    ELengthIval,
    Expr,
    NumTypes,
    EUndef = -1,
};

static const char *TypeCodes[] = {
    "RegMatch",
    "RegLength",
    "Regexpr",
    "ExprLength",
    "ExprLengthInterval",
    "Expression"
};

static void usage(const char *msg = nullptr, ...) {
  if (msg) {
    va_list args;
    va_start(args, msg);
    fprintf(stderr, "histtool: ");
    vfprintf(stderr, msg, args);
    fprintf(stderr, "\n");
  }
  fprintf(stderr, R"(usage: histtool <command> [options] <filenames...>

commands: makehistogram

makehistogram option:
    syntax: histtool makehistogram inputvalues.txt infile [outfile.txt]

)");
  exit(1);
}

ssize_t path_select(const std::vector<std::string> &values, const pbrt::path_entry &p) {
  return std::distance(values.begin(), std::find(values.begin(), values.end(), p.path));
}

ssize_t reg_select(const std::vector<std::string> &values, const pbrt::path_entry &p) {
  return std::distance(values.begin(), std::find_if(values.begin(), values.end(),
                                    [&](const std::string &r) { return std::regex_match(p.path, std::regex(r)); }));
}

ssize_t length_select(const std::vector<int> &values, const pbrt::path_entry &p) {
  return std::distance(values.begin(), std::find(values.begin(), values.end(), p.pathlen));
}

ssize_t lengthival_select(const std::vector<int> &values, const pbrt::path_entry &p) {
  for (auto it = values.begin(); it < (values.end() - 1); ++it) {
    if(p.pathlen >= *it && p.pathlen < *(it+1))
      return std::distance(values.begin(), it);
  }
  return values.size(); // If out of bounds
}

// Populate methods
void val_populate(std::vector<int> &values, const PathFile &file, const EType &e) {
  std::set<int> valueset;
  for(const pbrt::path_entry &p: file) {
    if(e == EType::ELengthIval || e == EType::ELength) {
      valueset.insert(int(p.pathlen));
    }
  }
  values.assign(valueset.begin(), valueset.end());
  std::sort(values.begin(), values.end());
}

void val_populate(std::vector<std::string> &values, const PathFile &file, const EType &e) {
  std::set<std::string> valueset;
  for(const pbrt::path_entry &p: file) {
    if (e == EType::RMatch) {
      valueset.insert(p.regex);
    } else if (e == EType::Expr) {
      valueset.insert(p.path);
    }
  }
  values.assign(valueset.begin(), valueset.end());
}

// Dispatcher

void fun_dispatcher(const EType &e, std::function<ssize_t(const std::vector<std::string>&, const pbrt::path_entry&)> &fun) {
  if (e == EType::RMatch) {
    fun = reg_select;
  } else if (e == EType::Expr) {
    fun = path_select;
  }
}

void fun_dispatcher(const EType &e, std::function<ssize_t(const std::vector<int>&, const pbrt::path_entry&)> &fun) {
  if(e == EType::ELengthIval) {
    fun = lengthival_select;
  } else if (e == EType::ELength) {
    fun = length_select;
  }
}

template <typename T>
std::vector<int> HistogramGenerator(const EType &e, std::vector<T> &values, const PathFile &file) {
  std::vector<int> hist;
  // If no values, populate w/ file data
  // TODO: populate and find frequencies in one pass
  if(values.empty()) {
    std::cout << "No values provided, populating from path file...";
    val_populate(values, file, e);
    std::cout << " done. " << values.size() << " distinct values found." << std::endl;
  }

  hist.resize(values.size() + 1);
  std::fill(hist.begin(), hist.end(), 0);

  std::function<ssize_t(const std::vector<T>&, const pbrt::path_entry&)> f;
  fun_dispatcher(e, f);

  for(const pbrt::path_entry &p : file) {
      ++hist[f(values, p)];
  }

  return hist;
}

EType typeParser(std::ifstream &is) {
  std::string str;
  std::getline(is, str);

  for (int i = 0; i < EType::NumTypes; ++i) {
    if(str.compare(TypeCodes[i]) == 0) { // if code found in the first word
      return (EType)i;
    }
  }

  return EType::EUndef;
}


void simple_histogram_output(std::ostream &os, const std::vector<std::string> &labels, const std::vector<int> &values) {
  os << "Histogram:" << std::endl;

  // Find maximum length for labels
  const int width = 20;

  std::vector<int> p(labels.size());
  // Lazy sorting of values
  std::iota(p.begin(), p.end(), 0);
  std::sort(p.begin(), p.end(),
                    [&](size_t i, size_t j){ return values[i] > values[j]; });

  for(int i = 0; i < p.size(); ++i) {
    os << std::left << std::setw(width) << labels[p[i]] << " : ";
    os << values[p[i]] << std::endl;
  }

  // Unsorted paths
  os << std::left << std::setw(width) << "Other " << " : ";
  os << values[labels.size()] << std::endl;
}

void histogram_generator(int argc, char* argv[]) {
  std::vector<int> histogram;
  // input file parsing
  std::ifstream paramfile;

  if(argc < 4) {
    usage("");
  }

  paramfile.open(argv[2], std::ios::in);
  // TODO: one vector ?
  std::vector<std::string> strvalues;
  PathFile infile(argv[3]);

  // Type
  EType type = typeParser(paramfile);
  if(type == EUndef) {
    usage("Undefined parameter type");
  }

  if(type == ELength || type == ELengthIval) {
    std::vector<int> intvalues;
    std::copy(std::istream_iterator<int>(paramfile), std::istream_iterator<int>(), std::back_inserter(intvalues));
    histogram = HistogramGenerator(type, intvalues, infile);
    // Conversion for labels (after b/c of label generation)
    std::for_each(intvalues.begin(), intvalues.end(), [&](int v) { strvalues.push_back(std::to_string(v)); });
  } else {
    std::copy(std::istream_iterator<std::string>(paramfile), std::istream_iterator<std::string>(),
              std::back_inserter(strvalues));
    histogram = HistogramGenerator(type, strvalues, infile);
  }

  if(argc == 5) {
    std::ofstream out;
    out.open(argv[4], std::ios::out);
    simple_histogram_output(out, strvalues, histogram);
  } else {
    simple_histogram_output(std::cout, strvalues, histogram);
  }

  exit(0);
}

int main(int argc, char* argv[]) {

  if(argc == 1) {
    usage("");
    return 1;
  }

  if(!strcmp(argv[1], "makehistogram")) {
    histogram_generator(argc, argv);
  } else {
    usage("");
  }

  return 0;
}
