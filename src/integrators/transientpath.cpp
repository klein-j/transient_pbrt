
// integrators/transientpath.cpp*
#include "integrators/transientpath.h"
#include "bssrdf.h"
#include "camera.h"
#include "films/transientfilm.h"
#include "interaction.h"
#include "paramset.h"
#include "scene.h"
#include "stats.h"
#include "progressreporter.h"
#include <limits>

namespace pbrt {

STAT_PERCENT("Integrator/Zero-radiance paths", zeroRadiancePaths, totalPaths);
STAT_INT_DISTRIBUTION("Integrator/Path length", pathLength);
STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);


// this function is an exact copy of EstimateDirect except for the part that computes the path length
std::pair<Float, Spectrum> TransientEstimateDirect(const Interaction &it, const Point2f &uScattering,
                        const Light &light, const Point2f &uLight,
                        const Scene &scene, Sampler &sampler,
                        MemoryArena &arena, bool handleMedia = false, bool specular = false) {
    BxDFType bsdfFlags =
        specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Spectrum Ld(0.f);
	Float GeometricPathLength = 0.f;
    // Sample light source with multiple importance sampling
    Vector3f wi;
    Float lightPdf = 0, scatteringPdf = 0;
    VisibilityTester visibility;
    Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
    VLOG(2) << "EstimateDirect uLight:" << uLight << " -> Li: " << Li << ", wi: "
            << wi << ", pdf: " << lightPdf;

	GeometricPathLength = (visibility.P0().p - visibility.P1().p).Length();

    if (lightPdf > 0 && !Li.IsBlack()) {
        // Compute BSDF or phase function's value for light sample
        Spectrum f;
        if (it.IsSurfaceInteraction()) {
            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->f(isect.wo, wi, bsdfFlags) *
                AbsDot(wi, isect.shading.n);
            scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, bsdfFlags);
            VLOG(2) << "  surf f*dot :" << f << ", scatteringPdf: " << scatteringPdf;
        } else {
            // Evaluate phase function for light sampling strategy
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;
            VLOG(2) << "  medium p: " << p;
        }
        if (!f.IsBlack()) {
            // Compute effect of visibility for light source sample
            if (handleMedia) {
                Li *= visibility.Tr(scene, sampler);
                VLOG(2) << "  after Tr, Li: " << Li;
            } else {
              if (!visibility.Unoccluded(scene)) {
                VLOG(2) << "  shadow ray blocked";
                Li = Spectrum(0.f);
              } else
                VLOG(2) << "  shadow ray unoccluded";
            }

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack()) {
                if (IsDeltaLight(light.flags))
                    Ld += f * Li / lightPdf;
                else {
                    Float weight =
                        PowerHeuristic(1, lightPdf, 1, scatteringPdf);
                    Ld += f * Li * weight / lightPdf;
                }
            }
        }
    }

    // Sample BSDF with multiple importance sampling
    if (!IsDeltaLight(light.flags)) {
        Spectrum f;
        bool sampledSpecular = false;
        if (it.IsSurfaceInteraction()) {
            // Sample scattered direction for surface interactions
            BxDFType sampledType;
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->Sample_f(isect.wo, &wi, uScattering, &scatteringPdf,
                                     bsdfFlags, &sampledType);
            f *= AbsDot(wi, isect.shading.n);
            sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;
        } else {
            // Sample scattered direction for medium interactions
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->Sample_p(mi.wo, &wi, uScattering);
            f = Spectrum(p);
            scatteringPdf = p;
        }
        VLOG(2) << "  BSDF / phase sampling f: " << f << ", scatteringPdf: " <<
            scatteringPdf;
        if (!f.IsBlack() && scatteringPdf > 0) {
            // Account for light contributions along sampled direction _wi_
            Float weight = 1;
            if (!sampledSpecular) {
                lightPdf = light.Pdf_Li(it, wi);
                if (lightPdf == 0) return std::make_pair(GeometricPathLength, Ld);
                weight = PowerHeuristic(1, scatteringPdf, 1, lightPdf);
            }

            // Find intersection and compute transmittance
            SurfaceInteraction lightIsect;
            Ray ray = it.SpawnRay(wi);
            Spectrum Tr(1.f);
            bool foundSurfaceInteraction =
                handleMedia ? scene.IntersectTr(ray, sampler, &lightIsect, &Tr)
                            : scene.Intersect(ray, &lightIsect);

            // Add light contribution from material sampling
            Spectrum Li(0.f);
            if (foundSurfaceInteraction) {
                if (lightIsect.primitive->GetAreaLight() == &light)
                    Li = lightIsect.Le(-wi);
            } else
                Li = light.Le(ray);
            if (!Li.IsBlack()) Ld += f * Li * Tr * weight / scatteringPdf;
        }
    }
    return std::make_pair(GeometricPathLength, Ld);
}

// this function is an exact copy of UniformSampleOneLight except for the part that computes the path length
std::pair<Float, Spectrum> TransientUniformSampleOneLight(const Interaction &it, const Scene &scene,
                               MemoryArena &arena, Sampler &sampler,
                               bool handleMedia, const Distribution1D *lightDistrib) {
    ProfilePhase p(Prof::DirectLighting);
    // Randomly choose a single light to sample, _light_
    int nLights = int(scene.lights.size());
    if (nLights == 0) return std::make_pair(0, Spectrum(0.f));
    int lightNum;
    Float lightPdf;
    if (lightDistrib) {
        lightNum = lightDistrib->SampleDiscrete(sampler.Get1D(), &lightPdf);
        if (lightPdf == 0) return std::make_pair(0, Spectrum(0.f));
    } else {
        lightNum = std::min((int)(sampler.Get1D() * nLights), nLights - 1);
        lightPdf = Float(1) / nLights;
    }
    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    Point2f uLight = sampler.Get2D();
    Point2f uScattering = sampler.Get2D();

	auto lightSample = TransientEstimateDirect(it, uScattering, *light, uLight,
                          scene, sampler, arena, handleMedia);
    return std::make_pair(lightSample.first, lightSample.second / lightPdf);
}




// TransientPathIntegrator Method Definitions
TransientPathIntegrator::TransientPathIntegrator(int maxDepth,
                               std::shared_ptr<const Camera> camera,
                               std::shared_ptr<Sampler> sampler,
                               const Bounds2i &pixelBounds,
							   std::unique_ptr<TransientFilm> film,
							   bool ignoreDistanceToCamera,
							   Float rrThreshold,
                               const std::string &lightSampleStrategy):
	maxDepth(maxDepth), camera(camera), sampler(sampler), pixelBounds(pixelBounds), ignoreDistanceToCamera(ignoreDistanceToCamera),
	rrThreshold(rrThreshold), lightSampleStrategy(lightSampleStrategy),
	film(move(film))
{
}

void TransientPathIntegrator::Preprocess(const Scene &scene, Sampler &sampler) {
    lightDistribution =
        CreateLightSampleDistribution(lightSampleStrategy, scene);
}


void TransientPathIntegrator::Li(const RayDifferential &r, const Scene &scene,
                            Sampler &sampler, MemoryArena &arena, std::function<void(Spectrum, Float)> AddSample,
                            int depth) const {
    ProfilePhase p(Prof::SamplerIntegratorLi);
    Spectrum beta(1.f);
	float geometricPathLength = 0.f; // which correlates with the travel time of the light
	Point3f lastPos = r.o;
    RayDifferential ray(r);
    bool specularBounce = false;
    int bounces;
    // Added after book publication: etaScale tracks the accumulated effect
    // of radiance scaling due to rays passing through refractive
    // boundaries (see the derivation on p. 527 of the third edition). We
    // track this value in order to remove it from beta when we apply
    // Russian roulette; this is worthwhile, since it lets us sometimes
    // avoid terminating refracted rays that are about to be refracted back
    // out of a medium and thus have their beta value increased.
    Float etaScale = 1;

    for (bounces = 0;; ++bounces) {
        // Find next path vertex and accumulate contribution
        VLOG(2) << "Path tracer bounce " << bounces << ", current geometricPathLength = " << geometricPathLength
                << ", beta = " << beta;

		lastPos = ray.o;

        // Intersect _ray_ with scene and store intersection in _isect_
        SurfaceInteraction isect;
        bool foundIntersection = scene.Intersect(ray, &isect);
		
		// compute the length:
		if( ! (ignoreDistanceToCamera && bounces==0))
			geometricPathLength += (isect.p-lastPos).Length();
		lastPos = isect.p;

        // Possibly add emitted light at intersection
        if (bounces == 0 || specularBounce) {
            // Add emitted light at path vertex or from the environment
            if (foundIntersection) {
				AddSample(beta*isect.Le(-ray.d), geometricPathLength);
            }
			// it does not make sense to filter infinite lights here, as they don't have a meaningful timestamp...
        }


        // Terminate path if ray escaped or _maxDepth_ was reached
        if (!foundIntersection || bounces >= maxDepth) break;

        // Compute scattering functions and skip over medium boundaries
        isect.ComputeScatteringFunctions(ray, arena, true);
        if (!isect.bsdf) {
            VLOG(2) << "Skipping intersection due to null bsdf";
            ray = isect.SpawnRay(ray.d);
            bounces--;
            continue;
        }

        const Distribution1D *distrib = lightDistribution->Lookup(isect.p);

        // Sample illumination from lights to find path contribution.
        // (But skip this for perfectly specular BSDFs.)
        if (isect.bsdf->NumComponents(BxDFType(BSDF_ALL & ~BSDF_SPECULAR)) >
            0) {
            ++totalPaths;
			auto lightSample = TransientUniformSampleOneLight(isect, scene, arena,
                                                       sampler, false, distrib);
            Spectrum Ld = beta * lightSample.second;
            VLOG(2) << "Sampled direct lighting Ld = " << Ld;
            if (Ld.IsBlack()) ++zeroRadiancePaths;
            CHECK_GE(Ld.y(), 0.f);
			AddSample(Ld, geometricPathLength+lightSample.first); // not sure, if it is better to always add it, or to check if it is not black
        }

        // Sample BSDF to get new path direction
        Vector3f wo = -ray.d, wi;
        Float pdf;
        BxDFType flags;
        Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf,
                                          BSDF_ALL, &flags);
        VLOG(2) << "Sampled BSDF, f = " << f << ", pdf = " << pdf;
        if (f.IsBlack() || pdf == 0.f) break;
        beta *= f * AbsDot(wi, isect.shading.n) / pdf;
        VLOG(2) << "Updated beta = " << beta;
        CHECK_GE(beta.y(), 0.f);
        DCHECK(!std::isinf(beta.y()));
        specularBounce = (flags & BSDF_SPECULAR) != 0;
        if ((flags & BSDF_SPECULAR) && (flags & BSDF_TRANSMISSION)) {
            Float eta = isect.bsdf->eta;
            // Update the term that tracks radiance scaling for refraction
            // depending on whether the ray is entering or leaving the
            // medium.
            etaScale *= (Dot(wo, isect.n) > 0) ? (eta * eta) : 1 / (eta * eta);
        }
        ray = isect.SpawnRay(wi);
		
        // currently we do not support SSS, so the code is removed
		// ...

        // Possibly terminate the path with Russian roulette.
        // Factor out radiance scaling due to refraction in rrBeta.
        Spectrum rrBeta = beta * etaScale;
        if (rrBeta.MaxComponentValue() < rrThreshold && bounces > 3) {
            Float q = std::max((Float).05, 1 - rrBeta.MaxComponentValue());
            if (sampler.Get1D() < q) break;
            beta /= 1 - q;
            DCHECK(!std::isinf(beta.y()));
        }
    }
    ReportValue(pathLength, bounces);
}



void TransientPathIntegrator::Render(const Scene &scene)
{
	//TODO: should we check here, whether we have exactly one light source? could this ever be a problem?

	Preprocess(scene, *sampler);
	// Render image tiles in parallel

	// Compute number of tiles, _nTiles_, to use for parallel rendering
	Bounds2i sampleBounds = film->GetSampleBounds();
	Vector2i sampleExtent = sampleBounds.Diagonal();
	const int tileSize = 16;
	Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
		(sampleExtent.y + tileSize - 1) / tileSize);
	ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");
	{
		ParallelFor2D([&](Point2i tile) {
			// Render section of image corresponding to _tile_

			// Allocate _MemoryArena_ for tile
			MemoryArena arena;

			// Get sampler instance for tile
			int seed = tile.y * nTiles.x + tile.x;
			std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);

			// Compute sample bounds for tile
			int x0 = sampleBounds.pMin.x + tile.x * tileSize;
			int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
			int y0 = sampleBounds.pMin.y + tile.y * tileSize;
			int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
			Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
			LOG(INFO) << "Starting image tile " << tileBounds;

			// Get _FilmTile_ for tile
			auto filmTile = film->GetFilmTile(tileBounds);

			// Loop over pixels in tile to render them
			for(Point2i pixel : tileBounds) {
				{
					ProfilePhase pp(Prof::StartPixel);
					tileSampler->StartPixel(pixel);
				}

				// Do this check after the StartPixel() call; this keeps
				// the usage of RNG values from (most) Samplers that use
				// RNGs consistent, which improves reproducability /
				// debugging.
				if(!InsideExclusive(pixel, pixelBounds))
					continue;

				do {
					// Initialize _CameraSample_ for current sample
					CameraSample cameraSample =
						tileSampler->GetCameraSample(pixel);

					// Generate camera ray for current sample
					RayDifferential ray;
					Float rayWeight =
						camera->GenerateRayDifferential(cameraSample, &ray);
					ray.ScaleDifferentials(
						1 / std::sqrt((Float)tileSampler->samplesPerPixel));
					++nCameraRays;

					/* the normal path tracer would just accumulate intensities from multiple paths and return
						their sum. As each different path has a different length, we can't do this any more.
						Thus we pass a lambda to the function, that allows to add contribution of all
						subpaths one at a time.
						The code is formatted in a way that makes it very similar
						to the original one. */
					// Evaluate radiance along camera ray
					if(rayWeight > 0) Li(ray, scene, *tileSampler, arena, [&](Spectrum L, Float distance)
					{
						// Issue warning if unexpected radiance value returned					
						if(L.HasNaNs()) {
							LOG(ERROR) << StringPrintf(
								"Not-a-number radiance value returned "
								"for pixel (%d, %d), sample %d. Setting to black.",
								pixel.x, pixel.y,
								(int)tileSampler->CurrentSampleNumber());
							L = Spectrum(0.f);
						}
						else if(L.y() < -1e-5) {
							LOG(ERROR) << StringPrintf(
								"Negative luminance value, %f, returned "
								"for pixel (%d, %d), sample %d. Setting to black.",
								L.y(), pixel.x, pixel.y,
								(int)tileSampler->CurrentSampleNumber());
							L = Spectrum(0.f);
						}
						else if(std::isinf(L.y())) {
							LOG(ERROR) << StringPrintf(
								"Infinite luminance value returned "
								"for pixel (%d, %d), sample %d. Setting to black.",
								pixel.x, pixel.y,
								(int)tileSampler->CurrentSampleNumber());
							L = Spectrum(0.f);
						}
						VLOG(1) << "Camera sample: " << cameraSample << " -> ray: " <<
							ray << " -> L = " << L;

						// Add camera ray's contribution to image
						filmTile->AddSample(cameraSample.pFilm, distance, L.y(), rayWeight);
					});

					// Free _MemoryArena_ memory from computing image sample
					// value
					arena.Reset();
				} while(tileSampler->StartNextSample());
			}
			LOG(INFO) << "Finished image tile " << tileBounds;

			// Merge image tile into _Film_
			film->MergeFilmTile(std::move(filmTile));
			reporter.Update();
		}, nTiles);
		reporter.Done();
	}
	LOG(INFO) << "Rendering finished";

	// Save final image after rendering
	film->WriteImage();
}


std::unique_ptr<TransientPathIntegrator> CreateTransientPathIntegrator(const ParamSet &params,
                                     std::shared_ptr<Sampler> sampler,
									 std::shared_ptr<const Camera> camera,
									 std::unique_ptr<TransientFilm> film) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    Float rrThreshold = params.FindOneFloat("rrthreshold", 1.);
    std::string lightStrategy =
        params.FindOneString("lightsamplestrategy", "spatial");

	bool ignoreDistanceToCamera = params.FindOneBool("ignoreDistanceToCamera", false);

	return std::make_unique<TransientPathIntegrator>(maxDepth, camera, sampler, pixelBounds, std::move(film), ignoreDistanceToCamera,
                              rrThreshold, lightStrategy);
}

}  // namespace pbrt
