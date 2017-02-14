//
// Created by Aron Monszpart on 19/01/17.
//

#ifndef ACQ_NORMALESTIMATION_HPP
#define ACQ_NORMALESTIMATION_HPP

#include "acq/normalEstimation.h"

#include "Eigen/Eigenvalues"        // SelfAdjointEigenSolver

#include <iostream>

namespace acq {

template <typename _NeighbourIdListT>
Eigen::Matrix <typename CloudT::Scalar, 3, 1>
calculatePointNormal(
    CloudT            const& cloud, // N x 3
    int               const  pointIndex,
    _NeighbourIdListT const& neighbourIndices
) {
    //! Floating point type
    typedef typename CloudT::Scalar Scalar;
    //! 3x1 vector type
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    //! 3x3 matrix type
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    // Covariance matrix initialized to 0-s:
    Matrix3 cov(Matrix3::Zero());

    // For each neighbouring point
    for (auto const neighbourIndex : neighbourIndices) {

        // Skip, if first neighbour is same point
        if (pointIndex == neighbourIndex) {
            continue;
        } //...if same index

        // Calculate relative vector to neighbour
        Vector3 const vToNeighbour =
            cloud.row(neighbourIndex) - cloud.row(pointIndex);

        // Sum up covariance
        cov += vToNeighbour * vToNeighbour.transpose();

    } //...For neighbours

    // Solve for neighbourhood smallest eigen value
    Eigen::SelfAdjointEigenSolver <Matrix3> es(cov);

    // Find index of smallest eigen value
    int const smallestEigenValueId = // 0, 1 2
        static_cast<int>(
            std::distance(
                es.eigenvalues().data(),
                std::min_element(
                    es.eigenvalues().data(),
                    es.eigenvalues().data() + cloud.cols()
                )
            )
        );

    // Return smallest eigen vector
    return es.eigenvectors()
             .col(smallestEigenValueId)
             .normalized();

} // calculatePointNormal()

template <typename _FacesT>
NeighboursT
calculateCloudNeighboursFromFaces(
    _FacesT   const& faces
) {
    // calculate neighbours from faces' edge lists
    NeighboursT neighbours;
    // for each face
    for (int row = 0; row != faces.rows(); ++row) {
        // for each face vertex
        for (int vxId = 0; vxId != faces.cols(); ++vxId) {
            // id of "incoming" edge's start vertex
            int const leftNeighbourId =
                (vxId != 0) ? vxId - 1
                           : faces.cols() - 1;
            // id of "outgoing" edge's end vertex
            int const rightNeighbourId =
                (vxId < faces.cols() - 1) ? vxId + 1
                                         : 0;
            // store vertex has left neighbour as neighbour
            neighbours[faces(row, vxId)].insert(
                faces(row, leftNeighbourId)
            );
            // store left neighbour has vertex as neighbour
            neighbours[faces(row, leftNeighbourId)].insert(
                faces(row, vxId)
            );
            // store vertex has right neighbour as neighbour
            neighbours[faces(row, vxId)].insert(
                faces(row, rightNeighbourId)
            );
            // store right neighbour has vertex as neighbour
            neighbours[faces(row, rightNeighbourId)].insert(
                faces(row, vxId)
            );
        } //...for each vertex in face
    } //...for each face

    return neighbours;
} //...orientCloudNormalsFromFaces()

template <typename _NormalsT, typename _FacesT>
int orientCloudNormalsFromFaces(
    _FacesT  const& faces,
   _NormalsT      & normals
) {
    return orientCloudNormals(
        calculateCloudNeighboursFromFaces(faces),
        normals
    );
} //...orientCloudNormalsFromFaces()

} //...ns acq

#endif //ACQ_NORMALESTIMATION_HPP
