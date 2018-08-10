
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
#include "stats.h"

namespace pbrt {

STAT_COUNTER("Intersections/Regular ray intersection tests",
             nIntersectionTests);
STAT_COUNTER("Intersections/Shadow ray intersection tests", nShadowTests);



std::string to_string(const pbrt::Vector3f& v)
{
	using std::to_string;
	return to_string(v.x) + "/" + to_string(v.y) + "/" + to_string(v.z);
}

std::vector<const Triangle*> InitializeNlosObjects(std::vector<std::shared_ptr<Primitive>> primitives)
{
	/*
	As an optimization, only those NlosObjects that are visible from a NlosReflector are added to the list.
	Each triangle represents a plane. If all points of all reflectors are behind this plane, the triangle is not visible.

	More precisely:
	- if all points are on the same side of the plane, the whole convex hull of those points is on one side of the plane
	- the reflector lies in the convex hull, thus all of it is on the same sides

	*/


	// pbrt actually stores all triangle vertices in world coordinates (page 155) - so no transformation need to be applied. (I did not know this for quite some time...)

	// fetch all Reflector triangles
	std::vector<Point3f> reflectorVertices; // usually there is only one very simple reflector, so the number of vertices should be <32.
	for(const auto& obj : primitives)
	{
		auto gp = dynamic_cast<const GeometricPrimitive*>(obj.get());
		if(gp)
		{
			auto triangleShape = dynamic_cast<const Triangle*>(gp->GetShape());
			if(triangleShape)
			{
				if(triangleShape->GetMesh()->objectSemantic == TriangleMesh::ObjectSemantic::NlosReflector)
				{
					auto& p = triangleShape->GetMesh()->p;
					auto& v = triangleShape->v;
					reflectorVertices.insert(reflectorVertices.end(), {p[v[0]], p[v[1]], p[v[2]]});
				}
			}
		}
	}


	// visibility test:
	auto CheckVisibility = [&reflectorVertices](const pbrt::Triangle* triangle)
	{
		auto& p = triangle->GetMesh()->p;
		auto& n = triangle->GetMesh()->n;
		auto& v = triangle->v;

		auto p0=Vector3f(p[v[0]]);
		auto p1=Vector3f(p[v[1]]);
		auto p2=Vector3f(p[v[2]]);
		auto n0=Vector3f(n[v[0]]);
		auto n1=Vector3f(n[v[1]]);
		auto n2=Vector3f(n[v[2]]);

		// compute plane
		auto N = Cross(p0-p1, p0-p2); // we could normalize n - but we do not have to :)
		auto offset = Dot(N, p0);

		if(Dot(p0+n0, N) < offset) // normal vector is facing downwards
		{
			//flip plane
			N = -N;
			offset = -offset;
		}

		if(Dot(p1+n1, N)<offset || Dot(p2+n2, N)<offset)
		{
			Error(("inconsistent triangle normals: ["+to_string(p0) + "] / ["+to_string(p1)+"] / ["+to_string(p2)+"]").c_str());
		}

		//test all reflector vertices against the plane:
		for(auto& v : reflectorVertices)
		{
			if(Dot(Vector3f(v), N) > offset)
				return true;
		}

		return false;
	};



	std::vector<const Triangle*> result;
	auto numberOfCulledPrimitives = 0u;
	// iterate over all objects in the aggregate, and save all NlosObject's
	for(const auto& obj : primitives)
	{
		auto gp = dynamic_cast<const GeometricPrimitive*>(obj.get());
		if(gp)
		{
			auto triangleShape = dynamic_cast<const Triangle*>(gp->GetShape());
			if(triangleShape)
			{
				if(triangleShape->GetMesh()->objectSemantic == TriangleMesh::ObjectSemantic::NlosObject)
				{
					if(CheckVisibility(triangleShape))
						result.emplace_back(triangleShape);
					else
						numberOfCulledPrimitives++;
				}
			}
		}
	}

	if(!reflectorVertices.empty() && result.empty())
	{
		Error("NLoS reflector but no NLoS objects present in scene");
		if(numberOfCulledPrimitives > 0)
			Error("All NLoS object primitives are facing away from the wall");
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
	return {sizes.data(), static_cast<int>(sizes.size())};
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
	ray.tMax = Infinity;
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
