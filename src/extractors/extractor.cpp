//
// Created by nispaur on 4/21/17.
//

#include "filters/box.h"
#include "scene.h"
#include "paramset.h"
#include "interaction.h"
#include "extractors/extractor.h"
#include "extractors/pathoutput.h"
#include "spectrum.h"

namespace pbrt {


void NContainer::Init(const RayDifferential &r, int depth, const Scene &scene) {
    this->depth = depth;
}

void NContainer::ReportData(const SurfaceInteraction &isect) {
    if(depth == 0) {
      n = Faceforward(isect.n, isect.wo);
    }
}

Spectrum NContainer::ToSample() const {
    Float rgb[3] = {(n.x*0.5f)+0.5f, (n.y*.5f)+0.5f, (n.z*.5f)+.5f};
    return RGBSpectrum::FromRGB(rgb);
}


std::shared_ptr<Container> NormalExtractor::GetNewContainer(const Point2f &p) const {
    return std::shared_ptr<Container>(new NContainer(p));
}

void ZContainer::Init(const RayDifferential &r, int depth, const Scene &scene) {
  rayorigin = r.o;
  this->depth = depth;
}

void ZContainer::ReportData(const SurfaceInteraction &isect) {
    // TODO: compute zscale constant once in extractorfunc
    const Float z = Vector3f(isect.p-rayorigin).Length();
    if(depth == 0) {
      const Float zscale = (zfar == znear) ? 1.f : znear / (znear - zfar);
      distance = zfar == 0.f ? znear/z : (-zfar*zscale)*(1/z) + zscale;
    }
}

Spectrum ZContainer::ToSample() const {
    return Spectrum(distance);
}


void AlbedoContainer::ReportData(const SurfaceInteraction &isect) {
  ProfilePhase p(Prof::ExtractorReport);
  Point2f dummy;

  if(depth == 0 && isect.bsdf) {
    rho = integrate ?
      isect.bsdf->rho(nSamples, wi.data(), wo.data(), bxdftype) :
      isect.bsdf->rho(isect.wo, 0, &dummy, BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE));
      // ignoring type (only lambertian reflection has a closed form)
  }
}

void AlbedoContainer::Init(const RayDifferential &r, int depth, const Scene &scene) {
  ProfilePhase pp(Prof::ExtractorInit);
  this->depth = depth;

  if(depth == 0 && integrate) {
    // Static Random Number Generator; same instance across containers
    static RNG rng;

    // Generate sample points for albedo calculation
    for (int i = 0; i < nSamples; ++i) {
      Float x = rng.UniformFloat();
      Float y = rng.UniformFloat();
      wi.push_back(Point2f(x, y));
      wo.push_back(Point2f(x, x));
    }
  }
}

Extractor *CreateNormalExtractor(const ParamSet &params, const Point2i &fullResolution,
                                 Float diagonal, const std::string &imageFilename) {
  std::string filename = params.FindOneString("outputfile", "");
  if (filename == "") filename = "normal_" + imageFilename;

  return new Extractor(new NormalExtractor(), new Film(
          fullResolution,
          Bounds2f(Point2f(0, 0), Point2f(1, 1)),
          std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
          diagonal, filename, 1.f));
}

Extractor *CreateZExtractor(const ParamSet &params, const Point2i &fullResolution,
                            Float diagonal, const std::string &imageFilename) {
  std::string filename = params.FindOneString("outputfile", "");
  if (filename == "") filename = "depth_" + imageFilename;

  Float znear = params.FindOneFloat("znear", 0.1f);
  Float zfar = params.FindOneFloat("zfar", 10000.f);

  return new Extractor(new ZExtractor(znear, zfar), new Film(
          fullResolution,
          Bounds2f(Point2f(0, 0), Point2f(1, 1)),
          std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
          diagonal, filename, 1.f));
}


Extractor *CreateAlbedoExtractor(const ParamSet &params, const Point2i &fullResolution,
                                 Float diagonal, const std::string &imageFilename) {
  std::string filename = params.FindOneString("outputfile", "");
  if (filename == "") filename = "albedo_" + imageFilename;


  bool integrateAlbedo = !params.FindOneBool("closedformonly", false);
  BxDFType type;

  type = params.FindOneBool("bsdftransmission", false) ? BSDF_TRANSMISSION : BSDF_REFLECTION;
  std::string bxdftype = params.FindOneString("bxdftype", "");

  if(bxdftype == "diffuse") {
    type = BxDFType(BSDF_DIFFUSE|type);
  } else if (bxdftype == "specular") {
    type = BxDFType(BSDF_SPECULAR|type);
  } else {
    type = BxDFType(BSDF_ALL);
  }

  int nbSamples = integrateAlbedo ? params.FindOneInt("samples", 10) : 0;

  return new Extractor(new AlbedoExtractor(type, integrateAlbedo, nbSamples), new Film(
          fullResolution,
          Bounds2f(Point2f(0, 0), Point2f(1, 1)),
          std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
          diagonal, filename, 1.f));
}


std::unique_ptr<Containers> ExtractorManager::GetNewContainer(const Point2f &p) {
  Containers *container = new Containers();
  for(Extractor *ext : extractors) {
    container->Add(ext->f->GetNewContainer(p));
  }

  return std::unique_ptr<Containers>(container);
}

std::unique_ptr<ExtractorTileManager> ExtractorManager::GetNewExtractorTile(const Bounds2i &tileBounds) {
  ExtractorTileManager *exttile = new ExtractorTileManager();
  for(uint i = 0; i < extractors.size(); ++i) {
    if(dispatchtable[i].first)
      exttile->Add(std::move(paths[dispatchtable[i].second]->GetPathTile()));
    else
      exttile->Add(std::move(films[dispatchtable[i].second]->GetFilmTile(tileBounds)));
  }

  return std::unique_ptr<ExtractorTileManager>(exttile);
}

void ExtractorManager::MergeTiles(std::unique_ptr<ExtractorTileManager> tiles) {
  for(uint i = 0; i < extractors.size(); ++i) {
    if(dispatchtable[i].first)
      paths[dispatchtable[i].second]->MergePathTile(std::move(tiles->GetPathTile(i)));
    else
      films[dispatchtable[i].second]->MergeFilmTile(std::move(tiles->GetFilmTile(i)));
  }
}

void ExtractorManager::WriteOutput(Float splatScale) {
  // TODO: Generic output
  for(uint i = 0; i < extractors.size(); ++i) {
    if(dispatchtable[i].first)
      paths[dispatchtable[i].second]->WriteFile();
    else
      films[dispatchtable[i].second]->WriteImage(splatScale);
  }
}

void ExtractorManager::AddSplats(const Point2f &pSplat, const Containers &containers) {
  for (uint i = 0; i < extractors.size(); ++i) {
    if(!dispatchtable[i].first)
      containers.AddSplats(i, pSplat, films[dispatchtable[i].second]);
  }
}

void ExtractorTileManager::AddSamples(const Point2f &pFilm,
                                      std::unique_ptr<Containers> containers, Float sampleWeight) {
  for (uint i = 0; i < dispatchtable.size(); ++i) {
    if(dispatchtable[i].first)
      pathtiles[dispatchtable[i].second]->AddSample(pFilm, containers->GetContainer(i));
    else
      filmtiles[dispatchtable[i].second]->AddSample(pFilm, containers->ToSample(i), sampleWeight);
  }
}


void Containers::Init(const RayDifferential &r, int depth, const Scene &scene) {
  for (std::shared_ptr<Container> c : containers) {
    c->Init(r,depth,scene);
  }
}

}