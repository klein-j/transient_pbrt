
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


// core/scene.cpp*
#include "scene.h"
#include "camera.h"
#include "film.h"
#include "sampler.h"
#include "parallel.h"
#include "progressreporter.h"
#include "stats.h"

namespace pbrt {

STAT_COUNTER("Intersections/Regular ray intersection tests",
             nIntersectionTests);
STAT_COUNTER("Intersections/Shadow ray intersection tests", nShadowTests);


std::vector<const Triangle*> InitializeNlosObjects(std::vector<std::shared_ptr<Primitive>> primitives)
{
	std::vector<const Triangle*> result;
	// iterate over all objects in the aggregate, and save all NlosObject's
	for(const auto& obj : primitives)
	{
		auto gp = dynamic_cast<const GeometricPrimitive*>(obj.get());
		if(gp)
		{
			auto triangleShape = dynamic_cast<const Triangle*>(gp->GetShape());
			if(triangleShape && triangleShape->GetMesh()->objectSemantic == TriangleMesh::ObjectSemantic::NlosObject)
			{
				result.emplace_back(triangleShape);
			}
		}
	}
	return result;
}

Distribution1D InitializeDistribution(std::vector<const Triangle*>& nlosObjects)
{
	std::vector<float> sizes;
	for(auto& o : nlosObjects)
	{
		sizes.push_back(o->Area());
	}
	return {sizes.data(), sizes.size()};
}

Scene::Scene(std::shared_ptr<Primitive> aggregate,
          const std::vector<std::shared_ptr<Light>> &lights,
		  std::vector<std::shared_ptr<Primitive>> primitives)
        : lights(lights),
		nlosObjects(InitializeNlosObjects(primitives)),
		nlosObjectsDistribution(InitializeDistribution(nlosObjects)),
		aggregate(aggregate) {
        // Scene Constructor Implementation
        worldBound = aggregate->WorldBound();
        for (const auto &light : lights) {
            light->Preprocess(*this);
            if (light->flags & (int)LightFlags::Infinite)
                infiniteLights.push_back(light);
        }
    }

// Scene Method Definitions
bool Scene::Intersect(const Ray &ray, SurfaceInteraction *isect) const {
    ++nIntersectionTests;
    DCHECK_NE(ray.d, Vector3f(0,0,0));
    return aggregate->Intersect(ray, isect);
}

bool Scene::IntersectP(const Ray &ray) const {
    ++nShadowTests;
    DCHECK_NE(ray.d, Vector3f(0,0,0));
    return aggregate->IntersectP(ray);
}

bool Scene::IntersectTr(Ray ray, Sampler &sampler, SurfaceInteraction *isect,
                        Spectrum *Tr) const {
    *Tr = Spectrum(1.f);
    while (true) {
        bool hitSurface = Intersect(ray, isect);
        // Accumulate beam transmittance for ray segment
        if (ray.medium) *Tr *= ray.medium->Tr(ray, sampler);

        // Initialize next ray segment or terminate transmittance computation
        if (!hitSurface) return false;
        if (isect->primitive->GetMaterial() != nullptr) return true;
        ray = isect->SpawnRay(ray.d);
    }
}

}  // namespace pbrt
