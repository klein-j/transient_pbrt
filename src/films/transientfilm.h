


#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif


#include "pbrt.h"
#include "geometry.h"
#include "spectrum.h"
#include "filter.h"
#include "stats.h"
#include "parallel.h"

namespace pbrt {


class TransientFilmTile;

struct TransientPixel {
	Float intensity;
	Float filterWeightSum;
};

	
// Please read the documentation, which explains in detail what this class does, how it is different from Film and how it fits in the rest of the renderer

class TransientFilm {
public:
	TransientFilm(const Point3i &resolution, const Bounds2f &cropWindow,
		std::unique_ptr<Filter> filter, Float diagonal,
		const std::string &filename, Float scale,
		Float maxSampleLuminance = Infinity);
	Bounds2i GetSampleBounds() const;
	Bounds2f GetPhysicalExtent() const;
	std::unique_ptr<TransientFilmTile> GetFilmTile(const Bounds2i &sampleBounds);
	void MergeFilmTile(std::unique_ptr<TransientFilmTile> tile);

	/// writes the transient image to the previously specified file
	void WriteImage();

	const Point3i fullResolution;
	const Float diagonal;
	std::unique_ptr<Filter> filter;
	const std::string filename;
	Bounds2i croppedPixelBounds;
private:
	std::vector<TransientPixel> pixels;
	static PBRT_CONSTEXPR int filterTableWidth = 16;
	Float filterTable[filterTableWidth * filterTableWidth];
	std::mutex mutex;
	const Float scale;
	const Float maxSampleLuminance;


	TransientPixel &GetPixel(const Point2i &p) {
		CHECK(InsideExclusive(p, croppedPixelBounds));
		int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
		int offset = (p.x - croppedPixelBounds.pMin.x) +
			(p.y - croppedPixelBounds.pMin.y) * width;
		return pixels[offset];
	}
};


class TransientFilmTile {
public:
	TransientFilmTile(const Bounds2i &pixelBounds, const Vector2f &filterRadius,
		const Float *filterTable, int filterTableSize,
		Float maxSampleLuminance)
		: pixelBounds(pixelBounds),
		filterRadius(filterRadius),
		invFilterRadius(1 / filterRadius.x, 1 / filterRadius.y),
		filterTable(filterTable),
		filterTableSize(filterTableSize),
		maxSampleLuminance(maxSampleLuminance) {
		pixels = std::vector<TransientPixel>(std::max(0, pixelBounds.Area()));
	}
	void AddSample(const Point2f &pFilm, Float L,
		Float sampleWeight = 1.) {
		ProfilePhase _(Prof::AddFilmSample);
		if(L > maxSampleLuminance)
			L = maxSampleLuminance;
		// Compute sample's raster bounds
		Point2f pFilmDiscrete = pFilm - Vector2f(0.5f, 0.5f);
		Point2i p0 = (Point2i)Ceil(pFilmDiscrete - filterRadius);
		Point2i p1 =
			(Point2i)Floor(pFilmDiscrete + filterRadius) + Point2i(1, 1);
		p0 = Max(p0, pixelBounds.pMin);
		p1 = Min(p1, pixelBounds.pMax);

		// Loop over filter support and add sample to pixel arrays

		// Precompute $x$ and $y$ filter table offsets
		int *ifx = ALLOCA(int, p1.x - p0.x);
		for(int x = p0.x; x < p1.x; ++x) {
			Float fx = std::abs((x - pFilmDiscrete.x) * invFilterRadius.x *
				filterTableSize);
			ifx[x - p0.x] = std::min((int)std::floor(fx), filterTableSize - 1);
		}
		int *ify = ALLOCA(int, p1.y - p0.y);
		for(int y = p0.y; y < p1.y; ++y) {
			Float fy = std::abs((y - pFilmDiscrete.y) * invFilterRadius.y *
				filterTableSize);
			ify[y - p0.y] = std::min((int)std::floor(fy), filterTableSize - 1);
		}
		for(int y = p0.y; y < p1.y; ++y) {
			for(int x = p0.x; x < p1.x; ++x) {
				// Evaluate filter value at $(x,y)$ pixel
				int offset = ify[y - p0.y] * filterTableSize + ifx[x - p0.x];
				Float filterWeight = filterTable[offset];

				// Update pixel values with filtered sample contribution
				TransientPixel &pixel = GetPixel(Point2i(x, y));
				pixel.intensity += L * sampleWeight * filterWeight;
				pixel.filterWeightSum += filterWeight;
			}
		}
	}
	TransientPixel &GetPixel(const Point2i &p) {
		CHECK(InsideExclusive(p, pixelBounds));
		int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
		int offset =
			(p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
		return pixels[offset];
	}
	const TransientPixel &GetPixel(const Point2i &p) const {
		CHECK(InsideExclusive(p, pixelBounds));
		int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
		int offset =
			(p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
		return pixels[offset];
	}
	Bounds2i GetPixelBounds() const { return pixelBounds; }

private:
	const Bounds2i pixelBounds;
	const Vector2f filterRadius, invFilterRadius;
	const Float *filterTable;
	const int filterTableSize;
	std::vector<TransientPixel> pixels;
	const Float maxSampleLuminance;
	friend class TransientFilm;
};

std::unique_ptr<TransientFilm> CreateTransientFilm(const ParamSet &params, std::unique_ptr<Filter> filter);

}  // namespace pbrt
