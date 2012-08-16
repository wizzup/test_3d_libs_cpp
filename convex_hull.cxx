// This program create 3d convex hull for given points
// dependencies:
//      CGAL   : for convex hull computation
//      PCL    : for reading pcd input

#include "file_util.h"

#include <CGAL/Homogeneous.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/convex_hull_incremental_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/Polyhedron_VRML_1_ostream.h> 
#include <CGAL/IO/Polyhedron_inventor_ostream.h> 

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

// floating point precission stuff
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz RT;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float RT;
#endif

typedef CGAL::Homogeneous<RT>                  K; // real value type
typedef K::Point_3                             Point_3; // point is vector (3) of real
typedef CGAL::Polyhedron_3<K>                 Polyhedron; // polyhedron also defined from real value position

typedef Polyhedron::Facet_iterator                   Facet_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

int main()
{
    // read input pcd file
    pcl::PointCloud<pcl::PointXYZ> cloud_in;

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("test.pcd", cloud_in) == -1)
    {
        PCL_ERROR("read file error");
        return -1;
    }else
        std::cout << "PCD cloud loaded " << cloud_in.width * cloud_in.height << " points" << std::endl;

    int num_points = cloud_in.points.size();

    std::cout << "saving input to ply " <<  std::endl;
    ply_write_pcl<pcl::PointXYZ>(cloud_in, "input.ply");

    std::vector<Point_3> V;
    std::cout << "converting vertex to CGAL format" << std::endl;
    for(int i=0; i<num_points;i++)
    {
        Point_3 p(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z);

        V.push_back(p);
    }

    Polyhedron P; // define polyhedron to hold convex hull

    // compute convex hull
    std::cout << "computing convex hull ... "  << num_points << " points" << std::endl;
    CGAL::convex_hull_3( V.begin(), V.end(), P);

    // output to openinventor file
    // std::ofstream ofs("output.iv");
    // CGAL::Inventor_ostream out(ofs);
    // out << P;

    std::cout << "saving output to ply " <<  std::endl;
    ply_write_cgal<K>(P, "output.ply");

    return 0;
}
