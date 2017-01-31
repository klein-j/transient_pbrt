


#include "transientfilm.h"
#include "paramset.h"
#include "imageio.h"
#include "stats.h"

namespace pbrt {

STAT_MEMORY_COUNTER("Memory/Film pixels", filmPixelMemory);

// Film Method Definitions
TransientFilm::TransientFilm(const Point3i &resolution, const Bounds2f &cropWindow,
	std::unique_ptr<Filter> filt, Float diagonal,
	const std::string &filename, Float scale, Float maxSampleLuminance)
	: fullResolution(resolution),
	diagonal(diagonal * .001),
	filter(std::move(filt)),
	filename(filename),
	scale(scale),
	maxSampleLuminance(maxSampleLuminance) {
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
	pixels = std::unique_ptr<TransientPixel[]>(new TransientPixel[croppedPixelBounds.Area()]);
	filmPixelMemory += croppedPixelBounds.Area() * sizeof(TransientPixel);

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
		tilePixelBounds, filter->radius, filterTable, filterTableWidth,
		maxSampleLuminance));
}

void TransientFilm::Clear() {
	for(Point2i p : croppedPixelBounds) {
		auto& pixel = GetPixel(p);
		for(int c = 0; c < 3; ++c)
			pixel.splatXYZ[c] = pixel.xyz[c] = 0;
		pixel.filterWeightSum = 0;
	}
}

void TransientFilm::MergeFilmTile(std::unique_ptr<TransientFilmTile> tile) {
	ProfilePhase p(Prof::MergeFilmTile);
	VLOG(1) << "Merging film tile " << tile->pixelBounds;
	std::lock_guard<std::mutex> lock(mutex);
	for(Point2i pixel : tile->GetPixelBounds()) {
		// Merge _pixel_ into _Film::pixels_
		const auto& tilePixel = tile->GetPixel(pixel);
		auto& mergePixel = GetPixel(pixel);
		Float xyz[3];
		tilePixel.contribSum.ToXYZ(xyz);
		for(int i = 0; i < 3; ++i) mergePixel.xyz[i] += xyz[i];
		mergePixel.filterWeightSum += tilePixel.filterWeightSum;
	}
}

void TransientFilm::SetImage(const Spectrum *img) const {
	int nPixels = croppedPixelBounds.Area();
	for(int i = 0; i < nPixels; ++i) {
		auto& p = pixels[i];
		img[i].ToXYZ(p.xyz);
		p.filterWeightSum = 1;
		p.splatXYZ[0] = p.splatXYZ[1] = p.splatXYZ[2] = 0;
	}
}

void TransientFilm::WriteImage(Float splatScale) {
	// Convert image to RGB and compute final pixel values
	LOG(INFO) <<
		"Converting image to RGB and computing final weighted pixel values";
	std::unique_ptr<Float[]> rgb(new Float[3 * croppedPixelBounds.Area()]);
	int offset = 0;
	for(Point2i p : croppedPixelBounds) {
		// Convert pixel XYZ color to RGB
		auto& pixel = GetPixel(p);
		XYZToRGB(pixel.xyz, &rgb[3 * offset]);

		// Normalize pixel with weight sum
		Float filterWeightSum = pixel.filterWeightSum;
		if(filterWeightSum != 0) {
			Float invWt = (Float)1 / filterWeightSum;
			rgb[3 * offset] = std::max((Float)0, rgb[3 * offset] * invWt);
			rgb[3 * offset + 1] =
				std::max((Float)0, rgb[3 * offset + 1] * invWt);
			rgb[3 * offset + 2] =
				std::max((Float)0, rgb[3 * offset + 2] * invWt);
		}

		// Add splat value at pixel
		Float splatRGB[3];
		Float splatXYZ[3] ={pixel.splatXYZ[0], pixel.splatXYZ[1],
			pixel.splatXYZ[2]};
		XYZToRGB(splatXYZ, splatRGB);
		rgb[3 * offset] += splatScale * splatRGB[0];
		rgb[3 * offset + 1] += splatScale * splatRGB[1];
		rgb[3 * offset + 2] += splatScale * splatRGB[2];

		// Scale pixel value by _scale_
		rgb[3 * offset] *= scale;
		rgb[3 * offset + 1] *= scale;
		rgb[3 * offset + 2] *= scale;
		++offset;
	}

	// Write RGB image
	LOG(INFO) << "Writing image " << filename << " with bounds " <<
		croppedPixelBounds;
	pbrt::WriteImage(filename, &rgb[0], croppedPixelBounds, fullResolution);
}



std::unique_ptr<TransientFilm> CreateTransientFilm(const ParamSet &params, std::unique_ptr<Filter> filter) {
	// Intentionally use FindOneString() rather than FindOneFilename() here
	// so that the rendered image is left in the working directory, rather
	// than the directory the scene file lives in.
	std::string filename = params.FindOneString("filename", "");
	if(PbrtOptions.imageFile != "") {
		if(filename != "") {
			Warning(
				"Output filename supplied on command line, \"%s\", ignored "
				"due to filename provided in scene description file, \"%s\".",
				PbrtOptions.imageFile.c_str(), filename.c_str());
		}
		else
			filename = PbrtOptions.imageFile;
	}
	if(filename == "") filename = "pbrt.exr";

	int xres = params.FindOneInt("xresolution", 256);
	int yres = params.FindOneInt("yresolution", 256);
	int tres = params.FindOneInt("tresolution", 512);
	if(PbrtOptions.quickRender) xres = std::max(1, xres / 4);
	if(PbrtOptions.quickRender) yres = std::max(1, yres / 4);
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

	Float scale = params.FindOneFloat("scale", 1.);
	Float diagonal = params.FindOneFloat("diagonal", 35.);
	Float maxSampleLuminance = params.FindOneFloat("maxsampleluminance",
		Infinity);
	return std::make_unique<TransientFilm>(Point3i(xres, yres, tres), crop, std::move(filter), diagonal,
		filename, scale, maxSampleLuminance);
}

}  // namespace pbrt
