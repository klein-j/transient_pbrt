#include "rng.h"

namespace pbrt
{

unsigned long long PCG32_DEFAULT_STATE = 0x853c49e6748fea9bULL;


void SetGlobalSeed(unsigned long long seed)
{
	// JK: i don't really know, what i am doing here...
	PCG32_DEFAULT_STATE = 0x853c49e6748fea9bULL * (seed+1);
}

}
