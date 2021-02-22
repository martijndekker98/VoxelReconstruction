#include <cstdlib>
#include <string>
#include <iostream>
#include <random>
#include <sstream>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "utilities/Background.h"

using namespace std;
using namespace nl_uu_science_gmt;


int main(
		int argc, char** argv)
{
	// init background stuff
    Background::findOrCreateBackground();

	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}
