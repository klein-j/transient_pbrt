#pragma once


#include "pbrt.h"
#include "integrator.h"


class TransientPathIntegrator : public SamplerIntegrator
{
public:
	Spectrum Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const;
	TransientPathIntegrator(int maxDepth, std::shared_ptr<const Camera> camera, std::shared_ptr<Sampler> sampler, const Bounds2i &pixelBounds) :
		SamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth)
	{ }
private:
	const int maxDepth;
};


TransientPathIntegrator* CreateTransientPathIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler, std::shared_ptr<const Camera> camera);
