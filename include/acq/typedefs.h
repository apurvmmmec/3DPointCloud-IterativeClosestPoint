//
// Created by Aron Monszpart on 19/01/17.
//

#ifndef ACQ_TYPEDEFS_H
#define ACQ_TYPEDEFS_H

#include "Eigen/Core"

// Use this, if using aligned vector types from Eigen
// See also
// http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html

//#include "Eigen/StdVector"
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d)

#include <map>
#include <set>

namespace acq {

//! Dynamically sized matrix of points in rows, a "point cloud".
typedef Eigen::MatrixXd CloudT;
//! Dynamically sized matrix of vectors in rows, a list of normals.
typedef Eigen::MatrixXd NormalsT;
//! Dynamically sized matrix of face vertex indices in rows.
typedef Eigen::MatrixXi FacesT;

/** \brief An associative storage of neighbour indices for point cloud
 * { pointId => [neighbourId_0, nId_1, ... nId_k-1] }
 */
typedef std::map<int, std::set<size_t> > NeighboursT;

} //...ns acq

#endif //ACQ_TYPEDEFS_H
