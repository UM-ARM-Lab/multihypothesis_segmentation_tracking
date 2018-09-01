//
// Created by arprice on 8/31/18.
//

#ifndef PROJECT_SHAPE_UTILS_IMPL_H
#define PROJECT_SHAPE_UTILS_IMPL_H

#include "mps_voxels/shape_utils.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>

#include <CGAL/convex_hull_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/Projection_traits_xy_3.h>

#include <octomap/octomap_types.h>
#include <octomap/OcTree.h>

#include <vector>


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
	using Scalar = std::remove_reference_t<typename std::remove_const_t<decltype(std::declval<PointT>().x())>>;

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

	if(const Point_3* p = CGAL::object_cast<Point_3>(&obj))
	{
		std::cout << "Point " << *p << std::endl;
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
	else if(const Polyhedron_3* poly = CGAL::object_cast<Polyhedron_3>(&obj))
	{
//		poly->size_of_vertices();
//		point_set.insert(poly->points_begin(), poly->points_end());
		std::map<Point_3, unsigned> vertex_map;
		std::shared_ptr<shapes::Mesh> m = std::make_shared<shapes::Mesh>(poly->size_of_vertices(), poly->size_of_facets());
		for (auto iter = poly->points_begin(); iter != poly->points_end(); ++iter)
		{
			const auto& pt = *iter;
			// Insert if missing
			auto lb = vertex_map.lower_bound(pt);
			if (lb == vertex_map.end() || (vertex_map.key_comp()(pt, lb->first)))
			{
				unsigned idx = vertex_map.size();
				vertex_map.insert(lb, {pt, idx});
				m->vertices[3*idx+0] = pt.x();
				m->vertices[3*idx+1] = pt.y();
				m->vertices[3*idx+2] = pt.z();
			}
		}

		int idx_count = 0;

		for (Polyhedron_3::Facet_const_iterator facet_iter = poly->facets_begin();
			facet_iter != poly->facets_end(); ++facet_iter)
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
		assert(idx_count == poly->size_of_facets()*3);

		return m;
	}
	else { throw std::runtime_error("Unknown object type."); }
};

template <typename PointContainerT, typename PointT>
std::shared_ptr<shapes::Mesh> ZABB(const PointContainerT& points, PointContainerT& supports)
{
	using Scalar = std::remove_reference_t<typename std::remove_const_t<decltype(std::declval<PointT>().x())>>;

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

//	CGAL::Object obj;
	std::vector<Point_3> hull_points;
	// CGAL::convex_hull_2
	CGAL::ch_bykat(cgal_points.begin(), cgal_points.end(), hull_points.begin(),
	               CGAL::Projection_traits_xy_3<K>::Traits());


	std::vector<Point_2> hull_points_2;
	for (const auto& pt : hull_points)
	{
		hull_points_2.emplace_back(Point_2(pt.x(), pt.y()));
	}

	std::vector<Point_2> box_points;

//	using ProjectionTraits = CGAL::Projection_traits_xy_3<K>;
//	using QuadTraits = CGAL::Optimisation::Min_quadrilateral_traits_wrapper<ProjectionTraits>;
	CGAL::min_rectangle_2(hull_points_2.begin(), hull_points_2.end(), box_points.begin());

	float min = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::lowest();
	for (const auto& pt : points)
	{
		min = std::min(min, pt.z());
		max = std::max(max, pt.z());
	}

	for (auto z : std::vector<float>{min, max})
	{
		for (const auto& pt : box_points)
		{
			supports.push_back({static_cast<Scalar>(pt.x()),
			                    static_cast<Scalar>(pt.y()),
			                    static_cast<Scalar>(z)});
		}
	}
}

#endif //PROJECT_SHAPE_UTILS_IMPL_H
