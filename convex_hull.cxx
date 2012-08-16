// This program create 3d convex hull for given points
// dependencies:
//      CGAL   : for convex hull computation and write openinventor (.iv) output
//      PCL    : for reading pcd input

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
typedef CGAL::Polyhedron_3< K>                 Polyhedron; // polyhedron also defined from real value position

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
        std::cout << "cloud loaded " << cloud_in.width * cloud_in.height << " points" << std::endl;

    int num_points = cloud_in.points.size();

    // snapshot input to ply format for using with other viewer
    // TODO: output to ply file (manaually, should consider using stanford ply lib when mesh has more complex data)
    std::ofstream in_ply("input.ply");
    in_ply << "ply" << std::endl; // magic word
    in_ply << "format ascii 1.0" << std::endl; // format and version
    in_ply << "comment generate by wizzup's convex_hull" << std::endl; // comment
    in_ply << "element vertex " << num_points << std::endl; // number of vertex in file 
    in_ply << "property float32 x" << std::endl; // vertex position property 
    in_ply << "property float32 y" << std::endl; // vertex position property 
    in_ply << "property float32 z" << std::endl; // vertex position property 
    in_ply << "end_header" << std::endl; // end of header

    // write vertex position to ply file
    // TODO: better format and alignment using boost::format
    for(int i=0; i<num_points;i++)
    {
        in_ply << cloud_in.points[i].x 
                << " " << cloud_in.points[i].y 
                << " " << cloud_in.points[i].z << std::endl;
    }

    std::vector<Point_3> V;

    std::cout << "converting ..." << std::endl;
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
    std::ofstream ofs("output.iv");
    CGAL::Inventor_ostream out(ofs);
    out << P;

    // FIXME: UNDONE: write output mesh in ply format
    // snapshot input to ply format for using with other viewer
    // TODO: output to ply file (manaually, should consider using stanford ply lib when mesh has more complex data)
    std::ofstream out_ply("output.ply");
    out_ply << "ply" << std::endl; // magic word
    out_ply << "format ascii 1.0" << std::endl; // format and version
    out_ply << "comment generate by wizzup's convex_hull" << std::endl; // comment
    out_ply << "element vertex " << num_points << std::endl; // number of vertex in file 
    out_ply << "property float32 x" << std::endl; // vertex position property 
    out_ply << "property float32 y" << std::endl; // vertex position property 
    out_ply << "property float32 z" << std::endl; // vertex position property 
    out_ply << "end_header" << std::endl; // end of header

    // write vertex position to ply file
    // TODO: better format and alignment using boost::format
    for(int i=0; i<num_points;i++)
    {
        out_ply << cloud_in.points[i].x 
                << " " << cloud_in.points[i].y 
                << " " << cloud_in.points[i].z << std::endl;
    }


    return 0;
}
