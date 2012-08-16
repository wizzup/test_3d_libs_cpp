// file utility function for working with different geometric data format
// FIXME: code can be cleaner if using traits class for PCL and CGAL vertex position

#include <fstream>

#include <pcl/point_types.h>

#include <CGAL/Polyhedron_3.h>

// write pcd vertex position to ply file
//
template<typename T>
int 
ply_write_pcl(pcl::PointCloud<T> &cloud, std::string file_name)
{
    // snapshot input to ply format for using with other viewer
    // TODO: this output to ply file manaually, should consider using stanford ply lib when mesh has more complex data
    std::ofstream ply_file(file_name.c_str());
    ply_file << "ply" << std::endl; // magic word
    ply_file << "format ascii 1.0" << std::endl; // format and version
    ply_file << "comment generate by wizzup's convex_hull" << std::endl; // comment
    ply_file << "element vertex " << cloud.points.size() << std::endl; // number of vertex in file 
    ply_file << "property float32 x" << std::endl; // vertex position property 
    ply_file << "property float32 y" << std::endl; // vertex position property 
    ply_file << "property float32 z" << std::endl; // vertex position property 
    ply_file << "end_header" << std::endl; // end of header

    // write vertex position to ply file
    // TODO: better format and alignment using boost::format
    for(unsigned int i = 0; i < cloud.points.size(); i++)
    {
        ply_file   << cloud.points[i].x 
            << " " << cloud.points[i].y 
            << " " << cloud.points[i].z << std::endl;
    }

    return 0;
}

// write CGAL vertex position to ply file
//
template<typename T>
int 
ply_write_cgal(CGAL::Polyhedron_3<T> &poly, std::string file_name)
{
    // snapshot input to ply format for using with other viewer
    // TODO: this output to ply file manaually, should consider using stanford ply lib when mesh has more complex data
    std::ofstream ply_file(file_name.c_str());
    ply_file << "ply" << std::endl; // magic word
    ply_file << "format ascii 1.0" << std::endl; // format and version
    ply_file << "comment generate by wizzup's convex_hull" << std::endl; // comment
    ply_file << "element vertex " << poly.size_of_vertices() << std::endl; // number of vertex in file 
    ply_file << "property float32 x" << std::endl; // vertex position property 
    ply_file << "property float32 y" << std::endl; // vertex position property 
    ply_file << "property float32 z" << std::endl; // vertex position property 
    ply_file << "end_header" << std::endl; // end of header

    // write vertex position to ply file
    // TODO: better format and alignment using boost::format
    typename CGAL::Polyhedron_3<T>::Vertex_iterator begin = poly.vertices_begin();

    for(; begin != poly.vertices_end(); ++begin)
    {
        // use CGAL homogenous coord here
        ply_file   << begin->point().hx()
            << " " << begin->point().hy() 
            << " " << begin->point().hz() << std::endl;
     }

    return 0;
}
