//
// Created by Aron Monszpart on 19/01/17.
//

#ifndef ACQ_NORMALESTIMATION_H
#define ACQ_NORMALESTIMATION_H

#include "acq/typedefs.h"
#include <limits.h>
#include <vector>

namespace acq {

/** \addtogroup NormalEstimation
 *  @{
 */

/** \brief Estimates the normal of a single point
 *         given its ID and the ID of its neighbours.
 *
 * \param[in] cloud             N x 3 matrix containing points in rows.
 * \param[in] pointIndex        Row-index of point.
 * \param[in] neighbourIndices  List of row-indices of neighbours.
 *
 * \return A 3D vector that is the normal of point with ID \p pointIndex.
 */
template <typename _NeighbourIdListT>
Eigen::Matrix <typename CloudT::Scalar, 3, 1>
calculatePointNormal(
    CloudT            const& cloud,
    int               const  pointIndex,
    _NeighbourIdListT const& neighbourIndices);


/** \brief Estimates the neighbours of all points in cloud
 *         returning \p k neighbours max each.
 *
 * \param[in] k         How many neighbours too look for in point.
 * \param[in] maxDist   Maximum distance between vertex and neighbour.
 * \param[in] maxLeafs  FLANN parameter, maximum kdTree depth.
 *
 * \return An associative container with the varying length lists of neighbours.
 */
NeighboursT
calculateCloudNeighbours(
    CloudT               const& cloud,
    int                  const  k,
    float                const  maxDist = std::sqrt(std::numeric_limits<float>::max()) - 1.f,
    int                  const  maxLeafs = 10);

/** \brief Estimates the normals of all points in cloud using \p k neighbours max each.
 *
 * \param[in] cloud      Input pointcloud, N x 3, N 3D points in rows.
 * \param[in] neighbours Precomputed lists of neighbour Ids.
 *
 * \return N x 3 3D normals, the normals of the points in \p cloud.
 */
NormalsT
calculateCloudNormals(
    CloudT               const& cloud,
    NeighboursT          const& neighbours);

/** \brief Breadth-first-search to orient normals consistently
 *         using the provided neighbourhood information.
 *
 * \param[in]     neighbours A directed list of neighbour indices.
 * \param[in,out] normals    The normals to possibly flip.
 *
 * \return The number of normals flipped.
 */
int
orientCloudNormals(
    NeighboursT const& neighbours,
    NormalsT         & normals);

/** \brief Traverses faces and records neighbourhood information using face edges.
 *
 * \tparam _FacesT Concept: acq::FacesT aka. Eigen::MatrixXi.
 *
 * \param faces Indices of vertices belonging to a face in each row.
 *
 * \return The directed neighbourhood information.
 */
template <typename _FacesT>
NeighboursT
calculateCloudNeighboursFromFaces(
    _FacesT   const& faces
);

/** \brief Estimates neighbourhood information from faces,
 *         and then consistently flips normals using BFS traversal.
 *
 * \tparam _NormalsT Concept: acq::NormalsT, aka. Eigen::MatrixXd.
 * \tparam _FacesT   Concept: acq::FacesT, aka. Eigen::MatrixXi.
 *
 * \param[in     ] faces  Face vertex indices in rows.
 * \param[in,out] normals The normals to possibly flip.
 * \return
 */
template <typename _NormalsT, typename _FacesT>
int
orientCloudNormalsFromFaces(
    _FacesT     const& faces,
    _NormalsT        & normals);

/** @} (NormalEstimation) */

} //...ns acq

#endif //ACQ_NORMALESTIMATION_H
