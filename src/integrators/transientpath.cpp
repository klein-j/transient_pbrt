

#include "transientpath.h"

#include "paramset.h"
#include "camera.h"
#include "scene.h"



Spectrum TransientPathIntegrator::Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const
{
	Spectrum result(0.001);

	SurfaceInteraction isect;
	auto foundIntersection = scene.Intersect(ray, &isect);
	if(foundIntersection)
	{
		auto dist = (isect.p-ray.o).Length();
		
		// uv mapping display:
		//result[0]=isect.uv.x;
		//result[1]=1;
		//result[2]=isect.uv.y;

		result *= dist;
	}

	return result;
}

TransientPathIntegrator* CreateTransientPathIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler, std::shared_ptr<const Camera> camera)
{
	auto maxDepth = params.FindOneInt("maxdepth", 1);

	// still unsure, what pixelBounds are, just copied this part from path.cpp :|
	int np;
	const int *pb = params.FindInt("pixelbounds", &np);
	Bounds2i pixelBounds = camera->film->croppedPixelBounds;
	if(pb) {
		if(np != 4)
			Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
			np);
		else {
			pixelBounds = Intersect(pixelBounds,
				Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
			if(pixelBounds.Area() == 0)
				Error("Degenerate \"pixelbounds\" specified.");
		}
	}
	return new TransientPathIntegrator(maxDepth, camera, sampler, pixelBounds);
}
