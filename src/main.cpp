#include <iostream>
#include <stdlib.h>
#include <ctype.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include "Utils/Utilities.h"
#include "Utils/Log.h"
#include "Utils/Timer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"




using namespace std;
using namespace pcl;
using namespace cv;
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

void project(const PointCloud<pcl::PointXYZ>::ConstPtr& cloud, string path, const string& axis);
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

	/* Statistical Filter */
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
	/* Voxel Filter */
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
	timer.reset();
	/* PassThroug Filter */
	if(console::find_argument(argc,argv, "-p") >= 0){
		axis = argv[console::find_argument(argc,argv,"-p") + 1];

		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (ptr_cloud);
		pass.setFilterFieldName (axis);
		pass.setFilterLimits (0.0, 1.0);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_filtered);

		configuration += "PassThroug\n";
		configuration += "axis: 					"+ axis +"\n";
		configuration += "range: 					"+ to_string(0.0) +","+ to_string(1.0) + "\n";
		configuration += "total point after filer: 	"+ to_string(cloud_filtered->height * cloud_filtered->width) +"\n";
		configuration += "Time to complete: 		"+ timer.report() +"\n";
		cout << configuration << endl;
		ptr_log->write(configuration);
	}

	/* Save */
	if(console::find_argument(argc,argv,"--save")>= 0){
		int how_many_files = atoi(argv[console::find_argument(argc,argv,"--save") + 2]);
		Utilities::writePCDFile(ptr_cloud, savePath, how_many_files );
	}
	ptr_log->close();
	return 0;
}
void project(const PointCloud<pcl::PointXYZ>::ConstPtr& cloud, string path, const string& axis){
	PointXYZ min, max;
	int width, height;
	getMinMax3D(*cloud, min, max);

	if(axis.compare("x") == 0){
		width = max.y;
		height = max.z;
	}
	else if(axis.compare("y") == 0){
		width = max.x;
		height = max.z;
	}else if(axis.compare("z") == 0){
		width = max.x;
		height = max.y;
	}

	width *= 100 + 50;
	height *= 100 + 50;
	Mat image (width, height, CV_8UC3, Scalar(0,0,255));
	for(PointCloud<PointXYZ>::const_iterator i = cloud->begin(); i < cloud->end(); i++){
		int x = i->x;
		int y = i->y;
		int z = i->z;
		cv::Point center (x,y);
		if(axis.compare("x") == 0){
			center.x = y;
			center.y = z;
		}else if(axis.compare("y") == 0){
			center.x = x;
			center.y = z;
		}else if(axis.compare("z") == 0){
			center.x = x;
			center.y = y;
		}
		center.x *= 100;
		center.y *= 100;
		circle(image, center, 1.0, Scalar(0,0,0));
	}
	string extension(Utilities::getExtension(path));
	path.replace(path.end()-3,path.end(),extension);
	imwrite( path + ".jpg", image);
}
