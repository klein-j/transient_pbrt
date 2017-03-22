
#include "transientfilm.h"
#include "paramset.h"
#include "imageio.h"
#include "stats.h"

#include <array>
#include <fstream>

namespace pbrt {

STAT_MEMORY_COUNTER("Memory/Film pixels", filmPixelMemory);

// Film Method Definitions
TransientFilm::TransientFilm(const Point3i &resolution, Float tmin, Float tmax,
	const Bounds2f &cropWindow,
	std::unique_ptr<Filter> filt, Float diagonal,
	const std::string &filename, Float scale, Float maxSampleLuminance)
	: fullResolution(resolution),
	tmin(tmin), tmax(tmax),
	diagonal(diagonal * .001),
	filter(std::move(filt)),
	filename(filename),
	scale(scale),
	maxSampleLuminance(maxSampleLuminance)
{
	// Compute film image bounds
	croppedPixelBounds =
		Bounds2i(Point2i(std::ceil(fullResolution.x * cropWindow.pMin.x),
		std::ceil(fullResolution.y * cropWindow.pMin.y)),
		Point2i(std::ceil(fullResolution.x * cropWindow.pMax.x),
		std::ceil(fullResolution.y * cropWindow.pMax.y)));
	LOG(INFO) << "Created film with full resolution " << resolution <<
		". Crop window of " << cropWindow << " -> croppedPixelBounds " <<
		croppedPixelBounds;

	// Allocate film image storage
	pixelIntensities.resize(croppedPixelBounds.Area() * fullResolution.z);
	pixelWeights.resize(croppedPixelBounds.Area());
	filmPixelMemory += croppedPixelBounds.Area() * fullResolution.z * sizeof(Float); // the intensities
	filmPixelMemory += croppedPixelBounds.Area() * sizeof(Float); // the weights

	// Precompute filter weight table
	int offset = 0;
	for(int y = 0; y < filterTableWidth; ++y) {
		for(int x = 0; x < filterTableWidth; ++x, ++offset) {
			Point2f p;
			p.x = (x + 0.5f) * filter->radius.x / filterTableWidth;
			p.y = (y + 0.5f) * filter->radius.y / filterTableWidth;
			filterTable[offset] = filter->Evaluate(p);
		}
	}
}

Bounds2i TransientFilm::GetSampleBounds() const {
	Bounds2f floatBounds(Floor(Point2f(croppedPixelBounds.pMin) +
		Vector2f(0.5f, 0.5f) - filter->radius),
		Ceil(Point2f(croppedPixelBounds.pMax) -
		Vector2f(0.5f, 0.5f) + filter->radius));
	return (Bounds2i)floatBounds;
}

Bounds2f TransientFilm::GetPhysicalExtent() const {
	Float aspect = (Float)fullResolution.y / (Float)fullResolution.x;
	Float x = std::sqrt(diagonal * diagonal / (1 + aspect * aspect));
	Float y = aspect * x;
	return Bounds2f(Point2f(-x / 2, -y / 2), Point2f(x / 2, y / 2));
}

std::unique_ptr<TransientFilmTile> TransientFilm::GetFilmTile(const Bounds2i &sampleBounds) {
	// Bound image pixels that samples in _sampleBounds_ contribute to
	Vector2f halfPixel = Vector2f(0.5f, 0.5f);
	Bounds2f floatBounds = (Bounds2f)sampleBounds;
	Point2i p0 = (Point2i)Ceil(floatBounds.pMin - halfPixel - filter->radius);
	Point2i p1 = (Point2i)Floor(floatBounds.pMax - halfPixel + filter->radius) +
		Point2i(1, 1);
	Bounds2i tilePixelBounds = Intersect(Bounds2i(p0, p1), croppedPixelBounds);
	return std::unique_ptr<TransientFilmTile>(new TransientFilmTile(
		tilePixelBounds, fullResolution.z, tmin, tmax, filter->radius, filterTable, filterTableWidth,
		maxSampleLuminance));
}

void TransientFilm::MergeFilmTile(std::unique_ptr<TransientFilmTile> tile) {
	ProfilePhase p(Prof::MergeFilmTile);
	VLOG(1) << "Merging film tile " << tile->pixelBounds;
	std::lock_guard<std::mutex> lock(mutex);
	for(Point2i pixel : tile->GetPixelBounds()) {

		// iterate over temporal dimension of pixels
		for(auto t=0; t<fullResolution.z; ++t) {
			// Merge _pixel_ into _Film::pixels_
			const auto& tilePixel = tile->GetPixel({pixel.x, pixel.y, t});
			auto& mergePixel = GetPixel({pixel.x, pixel.y, t});

			*mergePixel.intensity += *tilePixel.intensity;
			*mergePixel.filterWeightSum += *tilePixel.filterWeightSum;
		}
	}
}


// i wanted to have this as a local struct, but apparently a compiler bug in VC 12 prohibits it.
struct Header
{
	const std::array<char, 4> MagicValue = std::array<char, 4>({'T', 'I', '0', '0'});
	unsigned int xres=0, yres=0, tres=0;
	float tmin=0, tmax=0;
} header;

void TransientFilm::WriteImage() {
	// Convert image to RGB and compute final pixel values
	LOG(INFO) <<
		"Converting image to RGB and computing final weighted pixel values";

	std::vector<float> image(croppedPixelBounds.Area() * fullResolution.z);
	int offset = 0;
	for(Point2i p : croppedPixelBounds) {
		for(auto t=0; t<fullResolution.z; ++t)
		{
			auto pp= Point3i(p.x, p.y, t);
			// this is not quite right: if different samples fall in different times bins, the total amount is higher than if they fall into the same one
			// we have to add weights of all time bins and divide each bin by the total amount.
			// but how would this be changed, if we wanted also temporal sampling?
			image[offset] = *GetPixel(pp).intensity / *GetPixel(pp).filterWeightSum;
			++offset;
		}
	}

	// Write RGB image
	LOG(INFO) << "Writing image " << filename << " with bounds " <<
		croppedPixelBounds;

	using namespace std;
	fstream imageFile(filename, ios_base::out | ios_base::binary | ios_base::trunc);

	// write simple header:
	header.xres = (croppedPixelBounds.pMax-croppedPixelBounds.pMin).x;
	header.yres = (croppedPixelBounds.pMax-croppedPixelBounds.pMin).y;
	header.tres = fullResolution.z;
	header.tmin = tmin;
	header.tmax = tmax;

	imageFile.write((char*)&header, sizeof(Header));
	imageFile.write((char*)image.data(), sizeof(float)*croppedPixelBounds.Area() * fullResolution.z);
}

TransientPixelRef TransientFilm::GetPixel(const Point3i &p) {
	CHECK(InsideExclusive(Point2i(p.x, p.y), croppedPixelBounds));
	int width = croppedPixelBounds.pMax.x - croppedPixelBounds.pMin.x;
	int pixelOffset = (p.x - croppedPixelBounds.pMin.x) +
		(p.y - croppedPixelBounds.pMin.y) * width;
	return {&pixelIntensities[pixelOffset*fullResolution.z + p.z], &pixelWeights[pixelOffset]};
}




std::unique_ptr<TransientFilm> CreateTransientFilm(const ParamSet &params, std::unique_ptr<Filter> filter) {
	// Intentionally use FindOneString() rather than FindOneFilename() here
	// so that the rendered image is left in the working directory, rather
	// than the directory the scene file lives in.
	std::string filename = params.FindOneString("filename", "");
	if(PbrtOptions.imageFile != "") {
		filename = PbrtOptions.imageFile;
		if(filename != "") {
			Warning(
				"Output filename supplied in scene description file, \"%s\", ignored "
				"due to filename provided on command line, \"%s\".",
				filename.c_str(), PbrtOptions.imageFile.c_str());
		}
	}
	if(filename == "") filename = "pbrt.exr";

	int xres = params.FindOneInt("xresolution", 256);
	int yres = params.FindOneInt("yresolution", 256);
	int tres = params.FindOneInt("tresolution", 32);
	if(PbrtOptions.quickRender) xres = std::max(1, xres / 4);
	if(PbrtOptions.quickRender) yres = std::max(1, yres / 4);
	if(PbrtOptions.quickRender) tres = std::max(1, tres / 4);
	Bounds2f crop(Point2f(0, 0), Point2f(1, 1));
	int cwi;
	const Float *cr = params.FindFloat("cropwindow", &cwi);
	if(cr && cwi == 4) {
		crop.pMin.x = Clamp(std::min(cr[0], cr[1]), 0.f, 1.f);
		crop.pMax.x = Clamp(std::max(cr[0], cr[1]), 0.f, 1.f);
		crop.pMin.y = Clamp(std::min(cr[2], cr[3]), 0.f, 1.f);
		crop.pMax.y = Clamp(std::max(cr[2], cr[3]), 0.f, 1.f);
	}
	else if(cr)
		Error("%d values supplied for \"cropwindow\". Expected 4.", cwi);

	Float tmin = params.FindOneFloat("t_min", 0);
	Float tmax = params.FindOneFloat("t_max", 100);

	Float scale = params.FindOneFloat("scale", 1.);
	Float diagonal = params.FindOneFloat("diagonal", 35.);
	Float maxSampleLuminance = params.FindOneFloat("maxsampleluminance",
		Infinity);
	return std::make_unique<TransientFilm>(Point3i(xres, yres, tres), tmin, tmax, crop, std::move(filter), diagonal,
		filename, scale, maxSampleLuminance);
}





// ------------------   Transient Film Tile   ------------------------------

TransientFilmTile::TransientFilmTile(const Bounds2i &pixelBounds, unsigned int tresolution, Float tmin, Float tmax,
	const Vector2f &filterRadius,
		const Float *filterTable, int filterTableSize,
		Float maxSampleLuminance)
	: pixelBounds(pixelBounds),
	tresolution(tresolution),
	tmin(tmin), tmax(tmax),
	invBinSize(static_cast<Float>(tresolution)/(tmax-tmin)),
	filterRadius(filterRadius),
	invFilterRadius(1 / filterRadius.x, 1 / filterRadius.y),
	filterTable(filterTable),
	filterTableSize(filterTableSize),
	maxSampleLuminance(maxSampleLuminance)
{
	pixelIntensities = std::vector<Float>(std::max(0, pixelBounds.Area()) * tresolution);
	pixelWeights = std::vector<Float>(std::max(0, pixelBounds.Area()));
}


void TransientFilmTile::AddSample(const Point2f &pFilm, Float T, Float L, Float sampleWeight) {
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

	// same for temporal dimension
	auto timeBin = (T-tmin) * invBinSize;
	if(timeBin >= tresolution || timeBin < 0)
		return; //skip these samples

	auto tDiscrete = timeBin - 0.5;
	auto t0 = static_cast<int>( ceil(tDiscrete-filterRadius.x));
	auto t1 = static_cast<int>(floor(tDiscrete+filterRadius.x) + 1);
	t0 = std::max<int>(t0, 0);
	t1 = std::min<int>(t1, tresolution);

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

	// precompute temporal filter table offsets
	int *ift = ALLOCA(int, t1-t0);
	Float temporalFilterTotalWeight = 0;
	for(int t=t0; t<t1; ++t)
	{
		Float ft = std::abs((t - tDiscrete) * invFilterRadius.x *
			filterTableSize);
		auto offset = filterTableSize*static_cast<int>(filterTableSize/2); // to get the middle row of the filter
		ift[t - t0] = std::min((int)std::floor(ft), filterTableSize - 1) + offset;
		temporalFilterTotalWeight += filterTable[ift[t - t0]];
	}
	auto temporalFilterTotalWeightInv = 1.0 / temporalFilterTotalWeight;

	/*  normally the weight is stored for each pixel separately
		and at the end, the sample sum is divided by the sum of the weights.

		As we have importance sampling in the temporal dimension, we store
		the pixel weight only for the spatial dimensions. Thus the weights
		of filtering the temporal dimension is immediately applied and no
		denominator must be stored.
	*/
	for(int y = p0.y; y < p1.y; ++y) {
		for(int x = p0.x; x < p1.x; ++x) {
			// Evaluate filter value at $(x,y)$ pixel
			int offset = ify[y - p0.y] * filterTableSize + ifx[x - p0.x];
			Float filterWeight = filterTable[offset];
			
			for(int t=t0; t<t1; ++t)
			{
				// Update pixel values with filtered sample contribution
				auto pixel = GetPixel({x, y, t});
				*pixel.intensity += L * sampleWeight * filterWeight * filterTable[ift[t-t0]] * temporalFilterTotalWeightInv * invBinSize;
			}
			auto pixel = GetPixel({x, y, 0}); // filter weight sum is the same for all pixels with the same x,y
			*pixel.filterWeightSum += filterWeight;
		}
	}
}


TransientPixelRef TransientFilmTile::GetPixel(const Point3i &p) {
	CHECK(InsideExclusive(Point2i(p.x, p.y), pixelBounds));
	int width = pixelBounds.pMax.x - pixelBounds.pMin.x;
	int pixelOffset =
		(p.x - pixelBounds.pMin.x) + (p.y - pixelBounds.pMin.y) * width;
	
	return {&pixelIntensities[pixelOffset*tresolution + p.z], &pixelWeights[pixelOffset]};
}


Bounds2i TransientFilmTile::GetPixelBounds() const{
	return pixelBounds;
}

}  // namespace pbrt
