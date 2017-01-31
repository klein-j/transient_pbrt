


#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_TRANSIENTPATH_H
#define PBRT_INTEGRATORS_TRANSIENTPATH_H

// integrators/path.h*
#include "pbrt.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "films/transientfilm.h"

namespace pbrt
{


class TransientPathIntegrator : public Integrator
{
public:
    // TransientPathIntegrator Public Methods
	TransientPathIntegrator(int maxDepth,
	                        std::shared_ptr<const Camera> camera,
							std::shared_ptr<Sampler> sampler,
							const Bounds2i &pixelBounds,
							std::unique_ptr<TransientFilm> film,
							Float rrThreshold = 1,
							const std::string &lightSampleStrategy = "spatial");

	/// we don't strictly need this method (it is more of a interface), but we keep it
	/// so that our structure is closer to the original implementation.
    void Preprocess(const Scene &scene, Sampler &sampler);
	virtual Spectrum Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler,
	                    MemoryArena &arena, int depth = 0) const;
	virtual void Render(const Scene &scene);


private:
	const int maxDepth;
	std::shared_ptr<const Camera> camera;
	std::shared_ptr<Sampler> sampler;
	const Bounds2i pixelBounds;
	const Float rrThreshold;
	const std::string lightSampleStrategy;
	
	std::unique_ptr<LightDistribution> lightDistribution; // created during preprocessing

	std::unique_ptr<TransientFilm> film;
};

std::unique_ptr<TransientPathIntegrator> CreateTransientPathIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
									 std::shared_ptr<const Camera> camera,
									 std::unique_ptr<TransientFilm> film);


}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_TRANSIENTPATH_H
