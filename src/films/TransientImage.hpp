#pragma once


#include <string>
#include <array>
#include <vector>
#include <fstream>

namespace LibTransientImage
{


class Exception : public std::exception
{
public:
	Exception(std::string message): message(message) {}
	virtual const char* what() const noexcept override
	{ return message.c_str(); }
private:
	std::string message;
};



using MagicValue = std::array<char, 4>;
const MagicValue TiVersion00{'T', 'I', '0', '0'};
const MagicValue TiVersion01{'T', 'I', '0', '1'};
const MagicValue TiVersion04{'T', 'I', '0', '4'};


unsigned int ReadTransientImageFileVersion(std::string filename)
{
	MagicValue inputFile;

	std::ifstream file(filename, std::ios::binary);
	file.exceptions(std::ifstream::failbit);
	file.read(reinterpret_cast<char*>(&inputFile), sizeof(inputFile));

	if(inputFile == TiVersion00)
		return 0;
	if(inputFile == TiVersion01)
		return 1;
	if(inputFile == TiVersion04)
		return 4;

	throw Exception("Unknown File version: "+std::string(inputFile.begin(), inputFile.end()));
}

const unsigned int CurrentFileVersion = 4;

using Vec3 = std::array<float, 3>;



////////////////////////////////
//
//      Transient Image
//
////////////////////////////////

class T01
{
public:
	struct Header
	{
		std::array<char, 4> MagicValue;
		unsigned int numBins=0, uResolution=0, vResolution=0;
		float tmin=0, tmax=0;
	};

	T01();
	T01(std::string filename);
	void ReadFile(std::string filename);
	void WriteFile(std::string filename);
	float&       operator() (int t, int u, int v);
	const float& operator() (int t, int u, int v) const;
	float&       AccessPixel(int t, int u, int v);
	const float& AccessPixel(int t, int u, int v) const;

	Header header;
	std::vector<float> data;
	std::string imageProperties;
};

T01::T01()
{
	// initialize header
	header.MagicValue = {'T', 'I', '0', '1'};
	header.uResolution=header.vResolution=header.numBins=0;
	header.tmin=header.tmax = 0;
}

T01::T01(std::string filename)
{
	ReadFile(filename);
}

void T01::ReadFile(std::string filename)
{
	try
	{
		std::ifstream file(filename, std::ios::binary);
		file.exceptions(std::ifstream::failbit);
		file.read(reinterpret_cast<char*>(&header), sizeof(header));

		if("TI00" == std::string(header.MagicValue.data(), 4))
		{
			// convert flipped fields: XYT -> TXY
			auto h = header;
			header.numBins = h.uResolution;
			header.uResolution = h.vResolution;
			header.vResolution = h.numBins;
		}
		else if("TI01" == std::string(header.MagicValue.data(), 4))
		{
			// current version, everything is fine
		}
		else
			throw Exception("Header doesn't indicate supported transient image file: "+std::string(header.MagicValue.data(), 4));
		
		auto elements = header.uResolution*header.vResolution*header.numBins;
		data.resize(elements);
		file.read(reinterpret_cast<char*>(data.data()), sizeof(float)*elements);

		// count remaining bytes
		const auto currentFilePosition = file.tellg();
		file.seekg(0, std::ifstream::end);
		unsigned int propertiesLength = file.tellg()-currentFilePosition;
		file.seekg(currentFilePosition);

		// read image properties string
		imageProperties.resize(propertiesLength);
		file.read(reinterpret_cast<char*>(&imageProperties[0]), sizeof(char)*propertiesLength); // in C++11 std::string data is stored in a contiguous byte array
	}
	catch(std::exception &ex)
	{
		throw Exception("Exception reading " + filename + ": " + ex.what());
	}
}

void T01::WriteFile(std::string filename)
{
	try
	{
		std::ofstream file(filename, std::ios::binary);
		file.exceptions(std::ofstream::failbit);
		file.write(reinterpret_cast<char*>(&header), sizeof(header));
		const auto elements = header.uResolution*header.vResolution*header.numBins;
		file.write(reinterpret_cast<char*>(data.data()), sizeof(float)*elements);
		file.write(reinterpret_cast<const char*>(imageProperties.data()), sizeof(char)*imageProperties.length());
	}
	catch(std::exception &ex)
	{
		throw Exception("Exception writing " + filename + ": " + ex.what());
	}
}

float& T01::operator()(int t, int u, int v)
{
	return data[ t + header.numBins*(u + header.uResolution*(v))];
}
float& T01::AccessPixel(int t, int u, int v)
{
	return operator()(t,u,v);
}


const float& T01::operator()(int t, int u, int v) const
{
	return data[ t + header.numBins*(u + header.uResolution*(v))];
}
const float& T01::AccessPixel(int t, int u, int v) const
{
	return operator()(t, u, v);
}









class T04M10
{
public:

	struct FileHeader
	{
		std::array<char, 4> MagicValue = {'T', 'I', '0', '4'};
		unsigned int pixelMode = 0;
		unsigned int numPixels = 0;
		unsigned int numBins = 0;
		float tMin = 0;
		float tDelta = 0;
		unsigned int pixelInterpretationBlockSize = 0;
	};

	struct PixelInterpretationBlock
	{
		unsigned int uResolution = 0;
		unsigned int vResolution = 0;
		Vec3 topLeft = {0, 0, 0};
		Vec3 topRight ={0, 0, 0};
		Vec3 bottomLeft ={0, 0, 0};
		Vec3 bottomRight ={0, 0, 0};
		Vec3 laserPosition ={0, 0, 0};
	};

	T04M10();
	T04M10(std::string filename);

	/// Opens the file and selects appropriate loading function (ReadFileVersion01 or ReadFileVersion04, see below)
	void ReadFile(std::string filename);

	/// Writes the file in the 04 format
	void WriteFile(std::string filename);

	float&       operator() (int t, int u, int v);
	const float& operator() (int t, int u, int v) const;
	float&       AccessPixel(int t, int u, int v);
	const float& AccessPixel(int t, int u, int v) const;

	// file content
	FileHeader header;
	std::vector<float> data;
	PixelInterpretationBlock pixelInterpretationBlock;
	std::string imageProperties;

private:
	void ReadFileVersion01(std::ifstream& file);
	void ReadFileVersion04(std::ifstream& file);
};

T04M10::T04M10()
{
}

T04M10::T04M10(std::string filename)
{
	ReadFile(filename);
}

void T04M10::ReadFile(std::string filename)
{
	try
	{
		std::ifstream file(filename, std::ios::binary);
		file.exceptions(std::ifstream::failbit);

		// check version
		MagicValue version;
		file.read(reinterpret_cast<char*>(&version), sizeof(version));
		file.seekg(0); // jump back to start


		if(TiVersion00 == version)
		{
			ReadFileVersion01(file); //reads also the old format
		}
		else if(TiVersion01 == version)
		{
			ReadFileVersion01(file);
		}
		else if(TiVersion04 == version)
		{
			ReadFileVersion04(file);
		}
		else
			throw Exception("Wrong file version: "+std::string(header.MagicValue.begin(), header.MagicValue.end()));

	}
	catch(std::exception &ex)
	{
		throw Exception("Exception reading " + filename + ": " + ex.what());
	}
}


void T04M10::ReadFileVersion01(std::ifstream& file)
{
	T01::Header oldHeader;
	file.read(reinterpret_cast<char*>(&oldHeader), sizeof(oldHeader));

	if("TI00" == std::string(oldHeader.MagicValue.data(), 4))
	{
		// convert flipped fields: XYT -> TXY
		auto h = oldHeader;
		oldHeader.numBins = h.uResolution;
		oldHeader.uResolution = h.vResolution;
		oldHeader.vResolution = h.numBins;
	}
	else if("TI01" == std::string(oldHeader.MagicValue.data(), 4))
	{
		// expected version, everything is fine
	}
	else
		throw Exception("Header doesn't indicate supported transient image file: "+std::string(oldHeader.MagicValue.begin(), oldHeader.MagicValue.end()));


	auto numValues = oldHeader.uResolution*oldHeader.vResolution*oldHeader.numBins;
	
	
	// Image Header
	header.MagicValue = TiVersion04;
	header.pixelMode = 10;
	header.numPixels = oldHeader.uResolution*oldHeader.vResolution;
	header.numBins = oldHeader.numBins;
	header.tMin = oldHeader.tmin;
	header.tDelta = (float)(oldHeader.tmax-oldHeader.tmin) / (float)oldHeader.numBins;
	header.pixelInterpretationBlockSize = sizeof(PixelInterpretationBlock);


	// Pixel Data
	data.resize(numValues);
	file.read(reinterpret_cast<char*>(data.data()), sizeof(float)*numValues);

	// Pixel Interpretation
	pixelInterpretationBlock.uResolution = oldHeader.uResolution;
	pixelInterpretationBlock.vResolution = oldHeader.vResolution;
	// we assume the reflector is the XZ plane
	auto uResolution = static_cast<float>(oldHeader.uResolution);
	auto vResolution = static_cast<float>(oldHeader.vResolution);
	// we keep the corner points empty because we know nothing about the geometry

	// Image properties
	// count remaining bytes
	const auto currentFilePosition = file.tellg();
	file.seekg(0, std::ifstream::end);
	unsigned int propertiesLength = file.tellg()-currentFilePosition;
	file.seekg(currentFilePosition);

	// read image properties string
	imageProperties.resize(propertiesLength);
	file.read(reinterpret_cast<char*>(&imageProperties[0]), sizeof(char)*propertiesLength); // in C++11 std::string data is stored in a contiguous byte array

	// TODO: potentially add note to imageproperties, that this file was converted
}

void T04M10::ReadFileVersion04(std::ifstream& file)
{
	file.read(reinterpret_cast<char*>(&header), sizeof(header));

	if(10 != header.pixelMode)
			throw Exception("this class only support Mode 10 images");

	const auto numValues = header.numPixels*header.numBins;
	data.resize(numValues);
	file.read(reinterpret_cast<char*>(data.data()), sizeof(float)*numValues);
	file.read(reinterpret_cast<char*>(&pixelInterpretationBlock), sizeof(pixelInterpretationBlock));

	// count remaining bytes
	const auto currentFilePosition = file.tellg();
	file.seekg(0, std::ifstream::end);
	unsigned int propertiesLength = file.tellg()-currentFilePosition;
	file.seekg(currentFilePosition);

	// read image properties string
	imageProperties.resize(propertiesLength);
	file.read(reinterpret_cast<char*>(&imageProperties[0]), sizeof(char)*propertiesLength); // in C++11 std::string data is stored in a contiguous byte array
}


void T04M10::WriteFile(std::string filename)
{
	//sanity checks
	if(header.pixelMode != 10)
		throw Exception("erroneous pixelMode");
	if(header.pixelInterpretationBlockSize != sizeof(PixelInterpretationBlock))
		throw Exception("wrong PixelInterpretationBlockSize");

	try
	{
		std::ofstream file(filename, std::ios::binary);
		file.exceptions(std::ofstream::failbit);
		file.write(reinterpret_cast<char*>(&header), sizeof(header));
		const auto numValues = header.numPixels*header.numBins;
		file.write(reinterpret_cast<char*>(data.data()), sizeof(float)*numValues);
		file.write(reinterpret_cast<char*>(&pixelInterpretationBlock), sizeof(pixelInterpretationBlock));
		file.write(reinterpret_cast<const char*>(imageProperties.data()), sizeof(char)*imageProperties.length());
	}
	catch(std::exception &ex)
	{
		throw Exception("Exception writing " + filename + ": " + ex.what());
	}
}

float& T04M10::operator()(int t, int u, int v)
{
	return data[t + header.numBins*(u + pixelInterpretationBlock.uResolution*(v))];
}
float& T04M10::AccessPixel(int t, int u, int v)
{
	return operator()(t, u, v);
}


const float& T04M10::operator()(int t, int u, int v) const
{
	return data[t + header.numBins*(u + pixelInterpretationBlock.uResolution*(v))];
}
const float& T04M10::AccessPixel(int t, int u, int v) const
{
	return operator()(t, u, v);
}



// alias for the current file version
using TransientImage = T04M10;

}