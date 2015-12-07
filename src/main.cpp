#include <iostream>
#include <stdlib.h>
#include <ctype.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "Utils/Utilities.h"
#include "Utils/Log.h"
#include "Utils/Timer.h"


using namespace std;
using namespace pcl;
void printUsage(const char* name){
	cout << "Printing usage of: " <<name << endl
			<< "--file [path]				Read the file" << endl
			<< "--folder [path]				Read all pcd file of a folder" << endl
			<< "-s [media] [desvest]		(Optional) Statistical outlier removal, follow by: media and desvest" << endl
			<< "-v [size]					(Optional) Voxel grid, size: the size of the voxel" << endl
			<< "p [axis = z]				(Optional) Pass through filter, axis can be x, y or z, z is the default axis" << endl
			<< "--save [path] [how many] 	Save a pcd File"
			<< "-h							Print this Usage" << endl;
}

bool isAlpha(char str[]){
	int i = 0;
	while (str[i]){
		if (isalpha(str[i])){ return true;}
		i++;
	}
	return false;
}
int main(int argc, char** argv)
{
	if(argc <= 1 || console::find_argument(argc, argv, "-h") >= 0){
		printUsage(argv[0]);
	}
	//read file
	vector<string> paths;
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
	vector<PCLPointCloud2> cloud_blob;
	PointCloud<PointXYZ>::Ptr ptr_cloud (new PointCloud<PointXYZ>);
	Utilities::read(paths, cloud_blob);

	Utilities::convert2XYZ(cloud_blob, ptr_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	float media = 50, devest = 1.0, size;
	string axis ("z");

	string savePath;
	if(console::find_argument(argc,argv,"--save")>= 0){
		savePath = argv[console::find_argument(argc,argv,"--save") + 1];
	}

	Timer timer;
	Log* ptr_log;
	Log log(savePath);
	ptr_log = &log;
	string configuration("Filter:\n");
	if(console::find_argument(argc,argv,"-s")>= 0){

		if(!isAlpha(argv[console::find_argument(argc,argv,"-s") + 1])){
			media = atof(argv[console::find_argument(argc,argv,"-s") + 1]);
			devest = atof(argv[console::find_argument(argc,argv,"-s") + 2]);
		}

		StatisticalOutlierRemoval<PointXYZ> sor;
		sor.setInputCloud (ptr_cloud);
		sor.setMeanK (media);
		sor.setStddevMulThresh (devest);
		sor.filter (*cloud_filtered);

		configuration += "Statistical Outlier Removal\n";
		configuration += "media: 					"+ to_string(media) +"\n";
		configuration += "Desvest: 					"+ to_string(devest) +"\n";
		configuration += "total point after filer: 	"+ to_string(cloud_filtered->height * cloud_filtered->width) +"\n";
		configuration += "Time to complete: 		"+ timer.report() +"\n";
		cout << configuration << endl;
		ptr_log->write(configuration);
	}

	timer.reset();
	if(console::find_argument(argc,argv, "-v") >= 0){

		if(!isAlpha(argv[console::find_argument(argc,argv,"-v") + 1])){
			size = atof(argv[console::find_argument(argc,argv,"-v") + 1]);
		}
		// Create the filtering object
		VoxelGrid<PointXYZ> sor;
		sor.setInputCloud (ptr_cloud);
		sor.setLeafSize (0.01f, 0.01f, 0.01f);
		sor.filter (*cloud_filtered);

		configuration += "Voxel Grid\n";
		configuration += "size of voxel: 			"+ to_string(size) +"\n";
		configuration += "lief size: 				"+ to_string(0.01) +","+ to_string(0.01) +"," +to_string(0.01)+"\n";
		configuration += "total point after filer: 	"+ to_string(cloud_filtered->height * cloud_filtered->width) +"\n";
		configuration += "Time to complete: 		"+ timer.report() +"\n";
		cout << configuration << endl;
		ptr_log->write(configuration);

	}
	if(console::find_argument(argc,argv, "-p") >= 0){
		axis = argv[console::find_argument(argc,argv,"-p") + 1];
	}


	if(console::find_argument(argc,argv,"--save")>= 0){
		int how_many_files = atoi(argv[console::find_argument(argc,argv,"--save") + 2]);
		Utilities::writePCDFile(ptr_cloud, savePath, how_many_files );
	}
	ptr_log->close();
	return 0;
}
