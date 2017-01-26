
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

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

namespace pbrt
{


/**

TransientPathIntegrator is a combination of SamplerIntegrator and PathIntegrator.

At its core, it extends PathIntegrator to also compute the distances of the rays.
However, to increase efficiency, PathIntegrator::Li computes not only the light along
the complete path, but also computes direct illuminations of all sub paths. All
contributions are added and returned to SamplerIntegrator::Render. In our case however,
each subpath has a different length, and we need to return multiple samples, which is the
reason why we also have to change the SamplerIntegrator::Render method.

Apart from the Render method (which we need to change), SamplerIntegrator only has some
helper functions, that we do not use. Hence we combine our changes to PathIntegrator
and SamplerIntegrator to a single class TransientPathIntegrator.

**/


// TransientPathIntegrator Declarations
class TransientPathIntegrator : public Integrator
{
public:
    // TransientPathIntegrator Public Methods
	TransientPathIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
	               std::shared_ptr<Sampler> sampler,
	               const Bounds2i &pixelBounds, Float rrThreshold = 1,
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
};

TransientPathIntegrator *CreateTransientPathIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
                                     std::shared_ptr<const Camera> camera);


}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_TRANSIENTPATH_H
