// This program create 3d convex hull for given points
// dependencies:
//      CGAL   : for convex hull computation
//      PCL    : for reading pcd input
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <vector>

#include "file_util.h"

typedef CGAL::Simple_cartesian<double>               K; // real value type (double)
typedef K::Point_3                                   Point_3; // point is vector (3) of real
typedef CGAL::Polyhedron_3<K>                        Polyhedron; // polyhedron also defined from real value position

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

    std::cout << "converting vertex to CGAL format" << std::endl;
    std::vector<Point_3> V;
    pcl::PointCloud<pcl::PointXYZ>::iterator begin = cloud_in.points.begin();
    for(; begin != cloud_in.end(); ++begin)
    {
        Point_3 p(begin->x, begin->y, begin->z);
        V.push_back(p);
    }

    Polyhedron P; // define polyhedron to hold convex hull after computation

    // compute convex hull
    std::cout << "computing convex hull ... "  << num_points << " points" << std::endl;
    CGAL::convex_hull_3( V.begin(), V.end(), P);

    std::cout << "saving output to ply " <<  std::endl;
    ply_write_cgal<K>(P, "output.ply");

    return 0;
}
