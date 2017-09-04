//
// Path file manipulation tool
//

#ifndef PBRT_EXTLIB_PATHTOOL_H
#define PBRT_EXTLIB_PATHTOOL_H

#include "extractors/pathio.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>
#include <cstring>
#include <cerrno>

#define PAGE_SIZE (sysconf(_SC_PAGESIZE))
#define PG_RDOWN(x) (((long)(x)) & (~(PAGE_SIZE-1)))

// Path manipulation structs
struct vertex_entry {
    uint32_t type;  // 4
    std::array<float,3> v;     // 12
    std::array<float,3> n;     // 12
    std::array<float,3> bsdf;  // 12 (RGBSpectrum)
    float pdf_in;   // 4
    float pdf_out;  // 4
    // = 48 bytes
};

struct path_entry {

    uint32_t regexlen;                         // 4 o
    uint32_t pathlen;                          // 4 o
    char* regex;                               // regexlen o
    char* path;                                // pathlen o
    vertex_entry *vertices;         // 48 * pathlen o
    // = 8 + regexlen + (49 * pathlen) bytes

};

class PathFile {
  public:
    // Container typedefs

    template <typename T = pbrt::path_entry, typename A = std::allocator<T>>
    class pathconst_iterator {
      public:
        typedef std::size_t size_type;
        typedef typename A::difference_type difference_type;
        typedef typename A::value_type value_type;
        typedef typename A::reference const_reference;
        typedef typename A::pointer const_pointer;
        typedef std::forward_iterator_tag iterator_category;
        typedef PathFile vector_type;

        pathconst_iterator (const vector_type *vector_ptr, size_type offset = 0) : path_vector(vector_ptr) {
          cpos = (pbrt::path_entry*) vector_ptr->get_index(offset);
        }

        // TODO: make iterator copy constructible ?
        pathconst_iterator (const pathconst_iterator &it) : path_vector(it.path_vector), cpos(it.cpos) {}
        ~pathconst_iterator() {}

        pathconst_iterator& operator=(const pathconst_iterator& it) {
          path_vector = it.path_vector;
          cpos = it.cpos;
        }
        
        // TODO: full iterator content compare (ForwardIterator requirements)
        bool operator==(const pathconst_iterator &it) const { return cpos == it.cpos; }
        bool operator!=(const pathconst_iterator &it) const { return cpos != it.cpos; }
        bool operator<(const pathconst_iterator &it) const { return cpos < it.cpos; }
        bool operator>(const pathconst_iterator &it) const { return cpos > it.cpos; }
        bool operator<=(const pathconst_iterator &it) const { return cpos <= it.cpos; }
        bool operator>=(const pathconst_iterator &it) const { return cpos >= it.cpos; }

        pathconst_iterator& operator++() {
          cpos = next_pathptr(cpos);
          return *this;
        }

        const_reference operator++(int n) {
          pathconst_iterator it = pathconst_iterator(*this);
          cpos = next_pathptr(cpos);
          return *it;
        }

        pathconst_iterator& operator+=(size_type n) {
          for(int i = 0; i < n; ++i) {
            cpos = next_pathptr(cpos);
          }
          return *this;
        }

        // Bidirectionnal iterator operators, path table needed
        // const_iterator& operator--(); //optional
        // const_iterator operator--(int); //optional
        // const_iterator operator+(size_type) const; //optional
        // friend const_iterator operator+(size_type, const const_iterator&); //optional
        // const_iterator& operator-=(size_type); //optional
        // const_iterator operator-(size_type) const; //optional
        // difference_type operator-(const_iterator) const; //optional

        const_reference operator*() {
          return cpath = pbrt::path_entry::path_fromptr((void*)cpos);
        }

        const_pointer operator->() const {
          return cpos;
        }

        const_pointer get() const {
          return cpos;
        }

        // Random access
        const_reference operator[](pbrt::path_entry *pos) {
          return cpath = pbrt::path_entry::path_fromptr((void*)pos);
        }

      private:
        const_pointer next_pathptr(pbrt::path_entry* ptr) const {
          return (const_pointer)((char*)ptr + 2*sizeof(uint32_t) + 5*sizeof(float) + ptr->regexlen + (ptr->pathlen * (1 + sizeof(pbrt::vertex_entry))));
        }

        // Iterator attrs
        //pbrt::path_entry cpath;
        void *pathfile;
        pbrt::path_entry *cpos; // Current path position in file
        pbrt::path_entry cpath;

        const vector_type *path_vector;
    };

    // Container typedefs
    typedef std::size_t size_type;
    typedef PathFile::pathconst_iterator<pbrt::path_entry> const_iterator;

    PathFile(const std::string &filename) : fd(open(filename.c_str(), O_RDONLY)) {
      fstat(fd, &stats);
      std::cout << "Executing mmap with args: " << stats.st_size << " fd " << fd << std::endl;
      if((filemap = (int8_t*)mmap(nullptr, stats.st_size, MAP_SHARED, PROT_READ|MAP_NORESERVE, fd, 0)) == MAP_FAILED) {
        std::cerr << "mmap error "  << std::strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
      }

      // Read header and move map ptr to first path
      char *pathcountptr = strstr((char*)filemap, "Path file; n = ");
      pathcount = strtol(pathcountptr+15, nullptr, 10);
      first_path = (int8_t*)memchr(filemap, '\n', 80) + 1;
      make_index();
      // Place pointer to first path
      std::cout << "Loaded file, " << pathcount << " paths." << std::endl;
      std::cout << "First path pathlength = " << reinterpret_cast<pbrt::path_entry*>(first_path)->pathlen << std::endl;
      std::cout << "File loaded" << std::endl;
    }

    // iterator methods
    const_iterator begin() const {
      return PathFile::const_iterator(this, 0);
    }

    // past the end iterator
    const_iterator end() const {
      return PathFile::const_iterator(this, pathcount);
    }

    size_type size() const { 
      return pathcount; 
    }

    size_type average_length() const {
      uint64_t totallength = 0;
      for(const pbrt::path_entry &p : *this) {
        totallength += p.pathlen;
      }
      return !pathcount ? 0 : totallength/pathcount;
    }

    const pbrt::path_entry* get_index(size_type pos) const {
      return (pbrt::path_entry*)index[pos];
    }

    // Returns pointer to the first path
    const pbrt::path_entry* data() const {
      return (pbrt::path_entry*)first_path;
    }

    // Random access
    pbrt::path_entry operator[](size_type pos) const {
      return pbrt::path_entry::path_fromptr((void*)index[pos]);
    }

    bool eof() const {
      return current_pos >= pathcount;
    }

    ~PathFile() {
      munmap(filemap, stats.st_size);
      close(fd);
    }

  private:
    std::vector<size_type> make_index() {
      index.reserve(pathcount);
      pbrt::path_entry* ptr = (pbrt::path_entry*)first_path;
      for (int i = 0; i < pathcount; ++i) {
        index.push_back((uint64_t)ptr);
        ptr = (pbrt::path_entry*)((char*)ptr + 2*sizeof(uint32_t) + 5*sizeof(float) + ptr->regexlen + (ptr->pathlen * (1 + sizeof(pbrt::vertex_entry))));
      }
      index.push_back((uint64_t)ptr); // EOF ptr
      return index;
    }

    const int fd;
    struct stat stats;
    size_type pathcount;
    int8_t *filemap;
    int8_t *first_path;
    int current_pos;
    std::vector<size_type> index;
};

#endif //PBRT_EXTLIB_PATHTOOL_H
