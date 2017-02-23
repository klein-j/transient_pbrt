
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

/* A reference to a transient pixel. Each spatial pixel has only one weight,
thus there is a 3d intensity vector and a 2d weight vector. This struct
contains a intensity along with its shared weight.
*/
struct TransientPixelRef {
	Float* intensity;
	Float* filterWeightSum;
};

	
// Please read the documentation, which explains in detail what this class does, how it is different from Film and how it fits in the rest of the renderer

class TransientFilm {
public:
	TransientFilm(const Point3i &resolution, Float tmin, Float tmax,
		const Bounds2f &cropWindow,
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
	std::vector<Float> pixelIntensities;
	std::vector<Float> pixelWeights;
	Float tmin, tmax;


	static PBRT_CONSTEXPR int filterTableWidth = 16;
	Float filterTable[filterTableWidth * filterTableWidth];
	std::mutex mutex;
	const Float scale;
	const Float maxSampleLuminance;


	TransientPixelRef GetPixel(const Point3i &p);
};


class TransientFilmTile {
public:
	TransientFilmTile(const Bounds2i &pixelBounds, unsigned int tresolution, Float tmin, Float tmax,
		const Vector2f &filterRadius,
		const Float *filterTable, int filterTableSize,
		Float maxSampleLuminance);

	void AddSample(const Point2f &pFilm, Float t, Float L,
		Float sampleWeight = 1.);

	TransientPixelRef GetPixel(const Point3i &p);
	//const TransientPixelRef GetPixel(const Point3i &p) const;

	Bounds2i GetPixelBounds() const;
private:
	const Bounds2i pixelBounds;
	const unsigned int tresolution; ///< as the t dimension is never cropped, a Bounds3i would be pointless - thus we introduce this extra parameter
	const Float tmin, tmax;
	const Vector2f filterRadius, invFilterRadius;
	const Float *filterTable;
	const int filterTableSize;
	std::vector<Float> pixelIntensities;
	std::vector<Float> pixelWeights;
	const Float maxSampleLuminance;
	friend class TransientFilm;
};

std::unique_ptr<TransientFilm> CreateTransientFilm(const ParamSet &params, std::unique_ptr<Filter> filter);

}  // namespace pbrt
