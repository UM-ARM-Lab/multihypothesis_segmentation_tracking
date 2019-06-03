//
// Created by arprice on 8/31/18.
//

#ifndef MPS_SHAPE_UTILS_IMPL_H
#define MPS_SHAPE_UTILS_IMPL_H

#include "mps_voxels/shape_utils.h"
#include "mps_voxels/assert.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>

#include <CGAL/convex_hull_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/Projection_traits_xy_3.h>

#include <CGAL/Polygon_mesh_processing/orientation.h>
//#include <CGAL/boost/graph/property_maps.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/boost/graph/properties_Polyhedron_3.h>
//#include <CGAL/Polyhedron_items_with_id_3.h>

#include <octomap/octomap_types.h>
#include <octomap/OcTree.h>

#include <vector>
#include <type_traits>


using K =            CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_2 =      K::Point_2;
using Line_2 =       K::Line_2;
using Point_3 =      K::Point_3;
using Segment_3 =    K::Segment_3;
using Triangle_3 =   K::Triangle_3;
using Polygon_2 =    CGAL::Polygon_2<K>;
using Polyhedron_3 = CGAL::Polyhedron_3<K>;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

template <typename PointContainerT, typename PointT>
std::shared_ptr<shapes::Mesh> convex_hull(const PointContainerT& points)
{
//	using Scalar = std::remove_reference_t<typename std::remove_const_t<decltype(std::declval<PointT>().x())>>;

	if (points.empty())
	{
		return std::shared_ptr<shapes::Mesh>(nullptr);
	}

	std::vector<Point_3> cgal_points;
//	if (std::is_same<Point_3, PointT>::value)
//	{
//		cgal_points = points;
//	}
//	else
//	{
		for (const auto& pt : points)
		{
			cgal_points.emplace_back(Point_3(pt.x(), pt.y(), pt.z()));
		}
//	}

	CGAL::Object obj;
	CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), obj);

	std::set<Point_3> point_set;

	if(const Point_3* pt = CGAL::object_cast<Point_3>(&obj))
	{
		std::cout << "Point " << *pt << std::endl;
		return std::shared_ptr<shapes::Mesh>(nullptr);
	}
	else if(const Segment_3* s = CGAL::object_cast<Segment_3>(&obj))
	{
		std::cout << "Segment " << *s << std::endl;
		return std::shared_ptr<shapes::Mesh>(nullptr);
	}
	else if(const Triangle_3* t = CGAL::object_cast<Triangle_3>(&obj))
	{
		std::cout << "Triangle " << *t << std::endl;
		return std::shared_ptr<shapes::Mesh>(nullptr);
	}
	else if(const Polyhedron_3* p = CGAL::object_cast<Polyhedron_3>(&obj))
	{
		Polyhedron_3 poly = *p;
//		poly->size_of_vertices();
//		point_set.insert(poly->points_begin(), poly->points_end());

		if (!CGAL::Polygon_mesh_processing::is_outward_oriented(poly))
		{
//			CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3> polyGraph();
//			typedef typename CGAL::Polygon_mesh_processing::GetFaceIndexMap<Polyhedron_3,
//			                                 CGAL::cgal_bgl_named_params<bool, CGAL::internal_np::all_default_t>>::const_type Fid_map;
//			CGAL::cgal_bgl_named_params<bool, CGAL::internal_np::all_default_t> np = CGAL::parameters::all_default();
//			Fid_map fid_map = boost::choose_param(boost::get_param(np, CGAL::internal_np::face_index),
//			                               get_const_property_map(boost::face_index, poly));
//			std::vector<std::size_t> face_cc(CGAL::num_faces(poly), std::size_t(-1));
//			auto temp = CGAL::make_property_map(face_cc);
//			typedef typename boost::property_traits<Fid_map> Fid_traits;
//			Fid_traits::key_type;
//
//
//			typedef boost::graph_traits<Polyhedron_3>::face_descriptor face_descriptor;
//
//			auto temp2 = CGAL::bind_property_maps(fid_map, temp);
//			std::size_t nb_cc = boost::connected_components(poly,
//			                                                CGAL::bind_property_maps(fid_map, CGAL::make_property_map(face_cc)),
//			                                                CGAL::parameters::face_index_map(fid_map));

			// https://stackoverflow.com/questions/50199661/cgal-isotropic-remeshing-with-polyhedron-3/50207226
			typedef boost::graph_traits<Polyhedron_3>::face_descriptor face_descriptor;
			std::map<face_descriptor, std::size_t> fi_map;
			std::size_t id =0;
			BOOST_FOREACH(face_descriptor f, CGAL::faces(poly)) { fi_map[f]=id++; }
			CGAL::Polygon_mesh_processing::orient(poly,
			                                      CGAL::parameters::face_index_map(boost::make_assoc_property_map(fi_map))
				                                      .outward_orientation(true));
		}
		std::map<Point_3, unsigned> vertex_map;
		std::shared_ptr<shapes::Mesh> m = std::make_shared<shapes::Mesh>(poly.size_of_vertices(), poly.size_of_facets());
		for (auto iter = poly.points_begin(); iter != poly.points_end(); ++iter)
		{
			const auto& pnt = *iter;
			// Insert if missing
			auto lb = vertex_map.lower_bound(pnt);
			if (lb == vertex_map.end() || (vertex_map.key_comp()(pnt, lb->first)))
			{
				unsigned idx = vertex_map.size();
				vertex_map.insert(lb, {pnt, idx});
				m->vertices[3*idx+0] = pnt.x();
				m->vertices[3*idx+1] = pnt.y();
				m->vertices[3*idx+2] = pnt.z();
			}
		}

		int idx_count = 0;

		for (Polyhedron_3::Facet_const_iterator facet_iter = poly.facets_begin();
			facet_iter != poly.facets_end(); ++facet_iter)
		{
//			Polyhedron_3::Halfedge_const_handle h = facet.halfedge();
			Polyhedron_3::Halfedge_around_facet_const_circulator h = facet_iter->facet_begin();//facet.facet_begin();
			int vertex_count = 0;
			do
			{
				const Point_3& v = h->vertex()->point();
				unsigned idx = vertex_map.at(v);
				m->triangles[idx_count] = idx;
//				h = h->next();
				++vertex_count;
				++idx_count;
			} while (++h != facet_iter->facet_begin());

			if (3 != vertex_count)
				throw std::runtime_error("Facet contains " + std::to_string(vertex_count) + " vertices.");
		}
		MPS_ASSERT(idx_count == static_cast<int>(poly.size_of_facets())*3);

		m->computeTriangleNormals();
		return m;
	}
	else { throw std::runtime_error("Unknown object type."); }
}

template <typename PointContainerT, typename PointT>
std::shared_ptr<shapes::Mesh> prism(const PointContainerT& points)
{
	using Scalar = std::remove_reference_t<typename std::remove_const_t<decltype(std::declval<PointT>().x())>>;

	if (points.empty() || points.size() < 3)
	{
		return std::shared_ptr<shapes::Mesh>(nullptr);
	}

	std::vector<Point_2> cgal_points;
	for (const auto& pt : points)
	{
		cgal_points.emplace_back(Point_2(pt.x(), pt.y()));
	}

	std::vector<Point_2> hull_points;
	CGAL::convex_hull_2(cgal_points.begin(), cgal_points.end(), std::back_inserter(hull_points));
//	CGAL::ch_bykat(cgal_points.begin(), cgal_points.end(), std::back_inserter(hull_points));

	float min = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::lowest();
	for (const auto& pt : points)
	{
		min = std::min(min, pt.z());
		max = std::max(max, pt.z());
	}

	PointContainerT supports;
	for (auto z : std::vector<float>{min, max})
	{
		for (const auto& pt : hull_points)
		{
			supports.push_back({static_cast<Scalar>(pt.x()),
			                    static_cast<Scalar>(pt.y()),
			                    static_cast<Scalar>(z)});
		}
	}
	return convex_hull(supports);
}

template <typename PointContainerT, typename PointT>
std::shared_ptr<shapes::Mesh> ZAMBB(const PointContainerT& points)
{
	using Scalar = std::remove_reference_t<typename std::remove_const_t<decltype(std::declval<PointT>().x())>>;

	if (points.empty())
	{
		return std::shared_ptr<shapes::Mesh>(nullptr);
	}

	std::vector<Point_2> cgal_points;
	for (const auto& pt : points)
	{
		cgal_points.emplace_back(Point_2(pt.x(), pt.y()));
	}

	std::vector<Point_2> hull_points;
	CGAL::convex_hull_2(cgal_points.begin(), cgal_points.end(), std::back_inserter(hull_points));
//	CGAL::ch_bykat(cgal_points.begin(), cgal_points.end(), std::back_inserter(hull_points));

	std::vector<Point_2> box_points;
	CGAL::min_rectangle_2(hull_points.begin(), hull_points.end(), std::back_inserter(box_points));

	float min = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::lowest();
	for (const auto& pt : points)
	{
		min = std::min(min, pt.z());
		max = std::max(max, pt.z());
	}

	PointContainerT supports;
	for (auto z : std::vector<float>{min, max})
	{
		for (const auto& pt : box_points)
		{
			supports.push_back({static_cast<Scalar>(pt.x()),
			                    static_cast<Scalar>(pt.y()),
			                    static_cast<Scalar>(z)});
		}
	}
	return convex_hull(supports);
}

#endif //MPS_SHAPE_UTILS_IMPL_H
