//
// Created by bontius on 20/01/17.
//

#ifndef ACQ_DECORATEDCLOUD_H
#define ACQ_DECORATEDCLOUD_H

#include "acq/typedefs.h"

namespace acq {

/** \brief Simple class to keep track of points normals and faces for a point cloud or mesh. */
class DecoratedCloud {
public:
    /** \brief Default constructor leaving fields empty. */
    explicit DecoratedCloud() {}

    /** \brief Constructor filling point information only. */
    explicit DecoratedCloud(CloudT const& vertices);

    /** \brief Constructor filling point and face information. */
    explicit DecoratedCloud(CloudT const& vertices, FacesT const& faces);

    /** \brief Constructor filling point and normal information. */
    explicit DecoratedCloud(CloudT const& vertices, NormalsT const& normals);

    /** \brief Constructor filling point, face and normal information. */
    explicit DecoratedCloud(CloudT const& vertices, FacesT const& faces, NormalsT const& normals);

    /** \brief Getter for point cloud. */
    CloudT const& getVertices() const { return _vertices; }
    /** \brief Setter for point cloud. */
    void setVertices(CloudT const& vertices) { _vertices = vertices; }
    /** \brief Check, if any points stored. */
    bool hasVertices() const { return static_cast<bool>(_vertices.size()); }

    /** \brief Getter for face indices list. */
    FacesT const& getFaces() const { return _faces; }
    /** \brief Setter for face indices list. */
    void setFaces(FacesT const& faces) { _faces = faces; }
    /** \brief Check, if any faces stored. */
    bool hasFaces() const { return static_cast<bool>(_faces.size()); }

    /** \brief Getter for normals. */
    NormalsT      & getNormals() { return _normals; }
    /** \brief Getter for normals (const version). */
    NormalsT const& getNormals() const { return _normals; }
    /** \brief Setter for normals. */
    void setNormals(NormalsT const& normals) { _normals = normals; }
    /** \brief Check, if any normals stored. */
    bool hasNormals() const { return static_cast<bool>(_normals.size()); }

protected:
    CloudT   _vertices; //!< Point cloud, N x 3 matrix where N is the number of points.
    FacesT   _faces;    //!< Faces stored as rows of vertex indices (referring to \ref _vertices).
    NormalsT _normals;  //!< Per-vertex normals, associated with \ref _vertices by row ID.

public:
    // See https://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; //...class DecoratedCloud()

} //...ns acq

#endif //ACQ_DECORATEDCLOUD_H
