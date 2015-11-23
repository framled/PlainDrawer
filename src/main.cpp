#include <iostream>
#include <pcl/console/parse.h>

#include "utils/Utilities.h"
using namespace std;
using namespace pcl;
void printUsage(const char* name){
	cout << "Printing usage of: " <<name << endl
			<< "--file [path]			Read the file" << endl
			<< "--folder [path]			Read all pcd file of a folder" << endl
			<< "-s [media] [desvest]	(Optional) Statistical outlier removal, follow by: media and desvest" << endl
			<< "-v [size]				(Optional) Voxel grid, size: the size of the voxel" << endl
			<< "p [axis = z]			(Optional) Pass through filter, axis can be x, y or z, z is the default axis" << endl
			<< "-h						Print this Usage" << endl;
}
int main(int argc, char** argv)
{
	if(argc <= 1 || console::find_argument(argc, argv, "-h") >= 0){
		printUsage(argv[0]);
	}
	//read file
	if(console::find_argument(argc,argv,"--file")>= 0){
		vector<int> indices(pcl::console::parse_file_extension_argument(argc, argv, "pcd"));
		if (pcl::console::find_argument(argc, argv, "--save") >= 0){
			indices.erase(indices.end()-1);
		}

		Utilities::getFiles(argv, indices, paths);
		indices.clear();
		indices = pcl::console::parse_file_extension_argument(argc, argv, "ply");
		Utilities::getFiles(argv, indices, paths);
	}
	// or read a folder
	if(console::find_argument(argc,argv,"--folder")>= 0){
		Utilities::getFiles(argv[pcl::console::find_argument(argc, argv, "--folder") + 1], paths);
	}
	return 0;
}

