#include <cstdlib>
#include <string>
#include <iostream>
#include <random>
#include <sstream>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "utilities/Background.h"
#include "utilities/Calibrate.h"
#include "controllers/Camera.h"

using namespace std;
using namespace nl_uu_science_gmt;

const string datapath = "D:\\Master\\1e jaar\\Computer Vision\\Repos\\VoxelReconstruction\\data\\";


int main(
		int argc, char** argv)
{
	// init background stuff
    Background::findOrCreateBackground();

	//Calibrate::calibrate();

	vector<Camera*> cams;
	// camera init
	Camera camera1(datapath + "cam1\\", General::IntrinsicsFile, 1);
	camera1.initialize();
	
	Camera camera2(datapath + "cam2\\", General::IntrinsicsFile, 2);
	camera2.initialize();

	Camera camera3(datapath + "cam3\\", General::IntrinsicsFile, 3);
	camera3.initialize();

	Camera camera4(datapath + "cam4\\", General::IntrinsicsFile, 4);
	camera4.initialize();

	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}
