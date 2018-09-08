//
// Created by arprice on 9/7/18.
//

#ifndef MPS_VECTOR_LESS_THAN_H
#define MPS_VECTOR_LESS_THAN_H

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>>
struct vector_less_than
{
//	bool operator()(const PointT& a,
//	                const PointT& b) const
//	{
//		for(int i=0; i<DIM; ++i)
//		{
//			if(a[i]<b[i]) return true;
//			if(a[i]>b[i]) return false;
//		}
//		return false;
//	}
	bool operator()(const PointT& a,
	                const PointT& b) const
	{
		for(int i=0; i<DIM; ++i)
		{
			if(a(i)<b(i)) return true;
			if(a(i)>b(i)) return false;
		}
		return false;
	}
};

#endif //MPS_VECTOR_LESS_THAN_H
