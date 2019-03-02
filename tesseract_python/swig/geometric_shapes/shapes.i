
%{
#include <geometric_shapes/shapes.h>	
%}

%shared_ptr(shapes::Shape)
%shared_ptr(shapes::Sphere)
%shared_ptr(shapes::Cylinder)
%shared_ptr(shapes::Cone)
%shared_ptr(shapes::Box)
%shared_ptr(shapes::Mesh)
%shared_ptr(shapes::Plane)
%shared_ptr(shapes::OcTree)

%shared_factory(shapes::Shape, shapes::Sphere, shapes::Cylinder, shapes::Cone, shapes::Box, shapes::Mesh, shapes::Plane, shapes::OcTree);
%shared_factory(const shapes::Shape, const shapes::Sphere, const shapes::Cylinder, const shapes::Cone, const shapes::Box, const shapes::Mesh, const shapes::Plane, const shapes::OcTree);

namespace shapes
{
enum ShapeType
{
  UNKNOWN_SHAPE,
  SPHERE,
  CYLINDER,
  CONE,
  BOX,
  PLANE,
  MESH,
  OCTREE
};

%nodefaultctor Shape;
class Shape
{
public:
  
  virtual ~Shape();
  
  void scale(double scale);
  
  void padd(double padding);
  
  virtual void scaleAndPadd(double scale, double padd);
  
  virtual bool isFixed() const;
  
  ShapeType type;
  
  virtual Shape* clone() const;	 
 
};

class Sphere : public Shape
{
public:
  Sphere();
  
  Sphere(double r);
  
  static const std::string STRING_NAME;
      
  double radius;
};

class Cylinder : public Shape
{
public:
  Cylinder();
  
  Cylinder(double r, double l);
  
  static const std::string STRING_NAME;
  
  double length;
  double radius;  
};

class Cone : public Shape
{
public:
  Cone();
  Cone(double r, double l);
  
  static const std::string STRING_NAME;
  
  double length;
  double radius;  
};

class Box : public Shape
{
public:
  Box();
  Box(double x, double y, double z);

  static const std::string STRING_NAME;
    
  %extend
  {
	  Eigen::Vector3d _get_size()
	  {
		  return Eigen::Map<Eigen::Vector3d>($self->size);
	  }
	  
	  void _set_size(const Eigen::Ref<const Eigen::Vector3d>& size_in)
	  {
		  memcpy($self->size, size_in.data(), sizeof(double[3]));
	  }	  
  }
  
  %pythoncode %{
	  size=property(_get_size, _set_size)
  %}
  
  
};

class Mesh : public Shape
{
public:
  Mesh();
  Mesh(unsigned int v_count, unsigned int t_count);
  virtual ~Mesh();
  
  static const std::string STRING_NAME;
  
  void computeTriangleNormals();
  
  void computeVertexNormals();
  
  void mergeVertices(double threshold);
  
  /* 
   * Raw data stored in structure
   * 
   * unsigned int vertex_count;
   * double* vertices;
   * unsigned int triangle_count;
   * unsigned int* triangles;
   * double* triangle_normals;
   * double* vertex_normals; 
   */
  
  %extend
  {
	  Eigen::Matrix<double,3,Eigen::Dynamic> _get_vertices()
	  {
		return Eigen::Map<Eigen::Matrix3Xd>($self->vertices,3,$self->vertex_count);
	  }
	  
	  Eigen::Matrix<uint32_t,3,Eigen::Dynamic> _get_triangles()
	  {
		return Eigen::Map<Eigen::Matrix<unsigned int,3,Eigen::Dynamic>>($self->triangles,3,$self->triangle_count);
	  }
	  
	  Eigen::Matrix<double,3,Eigen::Dynamic> _get_triangle_normals()
	  {
		return Eigen::Map<Eigen::Matrix3Xd>($self->triangle_normals,3,$self->triangle_count);
	  }
	  
	  Eigen::Matrix<double,3,Eigen::Dynamic> _get_vertex_normals()
	  {
		return Eigen::Map<Eigen::Matrix3Xd>($self->vertex_normals,3,$self->vertex_count);
	  }
	  
  }
  
  %pythoncode %{
      vertices=property(_get_vertices)
      triangles=property(_get_triangles)
      triangle_normals=property(_get_triangle_normals)
      vertex_normals=property(_get_vertex_normals)    		
  %}  
  
};

class Plane : public Shape
{
public:
  Plane();
  Plane(double pa, double pb, double pc, double pd);
  
  static const std::string STRING_NAME;
  
  double a, b, c, d;
};

class OcTree : public Shape
{
public:
  OcTree();
  OcTree(const std::shared_ptr<const octomap::OcTree>& t);

  // Data stored in structure
  // std::shared_ptr<const octomap::OcTree> octree;
  
};

// Mesh creating helper functions

%{
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <octomap/OcTree.h>	
%}

Mesh* createMeshFromResource(const std::string& resource);
Mesh* createMeshFromResource(const std::string& resource, const Eigen::Vector3d& scale);
Mesh* createMeshFromShape(const Shape* shape);
 
}

%inline
{
	//Mesh* createMeshFromVertices(const EigenSTL::vector_Vector3d& vertices, const std::vector<unsigned int>& triangles);
	//Mesh* createMeshFromVertices(const EigenSTL::vector_Vector3d& source);
	
	shapes::Mesh* createMeshFromVertices(const Eigen::Ref<const Eigen::Matrix3Xd> & vertices, const Eigen::Ref<const Eigen::Matrix<uint32_t,3,Eigen::Dynamic>>& triangles)
	{
		EigenSTL::vector_Vector3d vertices2(vertices.cols());
		for (size_t i=0; i<vertices.cols(); i++)
		{
			vertices2[i] = vertices.block<3,1>(0,i);
		}
		std::vector<unsigned int> triangles2(triangles.cols()*3);
		for (size_t i=0; i<triangles.rows(); i++)
		{
			triangles2[i*3]     = triangles(0,i);
			triangles2[i*3 + 1] = triangles(1,i);
			triangles2[i*3 + 2] = triangles(2,i);
		}
		
		return shapes::createMeshFromVertices(vertices2, triangles2);
	}
	
	shapes::Mesh* createMeshFromVertices(const Eigen::Ref<const Eigen::Matrix3Xd> & source)
	{
		EigenSTL::vector_Vector3d source2(source.cols());
		for (size_t i=0; i<source.cols(); i++)
		{
			source2[i] = source.block<3,1>(0,i);
		}
					
		return shapes::createMeshFromVertices(source2);
	}
	
	std::shared_ptr<shapes::OcTree> loadOcTreeFromString(const std::string& data)
	{
		std::stringstream is(data);
		octomap::OcTree* octree=dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(is));
		if (!octree) throw std::runtime_error("Could not load octree");
		return std::make_shared<shapes::OcTree>(std::shared_ptr<octomap::OcTree>(octree));		
	}
	
	std::shared_ptr<shapes::OcTree> loadOcTreeFromFile(const std::string& filename)
	{		
		octomap::OcTree* octree=dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(filename));
		if (!octree) throw std::runtime_error("Could not load octree");
		return std::make_shared<shapes::OcTree>(std::shared_ptr<octomap::OcTree>(octree));		
	}

}

%rosmsg_typemaps(shape_msgs::SolidPrimitive);

namespace shapes
{
	Shape* constructShapeFromMsg(const shape_msgs::SolidPrimitive& shape_msg);
	Shape* constructShapeFromMsg(const shape_msgs::Plane& shape_msg);
	Shape* constructShapeFromMsg(const shape_msgs::Mesh& shape_msg);
}

%inline
{
	void constructSolidPrimitiveMsgFromShape(std::shared_ptr<shapes::Shape> shape, shape_msgs::SolidPrimitive& shape_msg)
	{
		shapes::ShapeMsg shape_msg1;
		if (!shapes::constructMsgFromShape(shape.get(), shape_msg1))
		{
			throw std::runtime_error("constructMsgFromShape failed");
		}
		shape_msg=boost::get<shape_msgs::SolidPrimitive>(shape_msg1);
	}
	
	void constructPlaneMsgFromShape(std::shared_ptr<shapes::Shape> shape, shape_msgs::Plane& shape_msg)
	{
		shapes::ShapeMsg shape_msg1;
		if (!shapes::constructMsgFromShape(shape.get(), shape_msg1))
		{
			throw std::runtime_error("constructMsgFromShape failed");
		}
		shape_msg=boost::get<shape_msgs::Plane>(shape_msg1);
	}
	
	void constructMeshMsgFromShape(std::shared_ptr<shapes::Shape> shape, shape_msgs::Mesh& shape_msg)
	{
		shapes::ShapeMsg shape_msg1;
		if (!shapes::constructMsgFromShape(shape.get(), shape_msg1))
		{
			throw std::runtime_error("constructMsgFromShape failed");
		}
		shape_msg=boost::get<shape_msgs::Mesh>(shape_msg1);
	}	
}

