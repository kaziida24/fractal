// Main program to view point cloud 

// Includes: 
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
    
int user_data;

using namespace std;

string cloudName;

string help_flag = "-h"; 

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    // std::cout << "i only run once" << std::endl;
    
}
    
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}

void showHelp()
{
    printf("\n"); 
    printf("Welcome to the help menu for the executable, visualize.\n");
    printf("This executable plots the specified PCD file using PCL's native viewer.\n"); 
    printf("\n"); 

    printf("Usage:\n"); 
    printf("1. Visualize specified PCD file.\n"); 
    printf("   ./visualize filename.pcd\n"); 
    printf("\n"); 
    printf("2. Access this help message again.\n"); 
    printf("   ./visualize -h\n"); 
    printf("\n");  
    return;
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "Check usage again." << endl; 
        return -1; 
    }
    else
    {
        if (!strcmp(argv[1], help_flag.c_str()))
        {
            showHelp(); 
            return -2; 
        }
        else
        {
            cloudName = argv[1]; 
        }
    }

    printf("Rendering %s using the PCL native viewer.\n", cloudName.c_str()); 

    string pointCloudName = "../datasets/"+cloudName; 

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (pointCloudName.c_str(), *cloud);

    cout << "Width: " << cloud->width << " Height: " << cloud->height << endl;
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    // //you can also do cool processing here
    // //FIXME: Note that this is running in a separate thread from viewerPsycho
    // //and you should guard against race conditions yourself...
    user_data++;
    }


    return 0;
}

