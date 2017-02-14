//
// Created by bontius on 20/01/17.
//

#include "acq/impl/decoratedCloud.hpp"

namespace acq {

DecoratedCloud::DecoratedCloud(CloudT const& vertices)
    : _vertices(vertices) {}

DecoratedCloud::DecoratedCloud(CloudT const& vertices, FacesT const& faces)
    : _vertices(vertices), _faces(faces)
{}

DecoratedCloud::DecoratedCloud(CloudT const& vertices, FacesT const& faces, NormalsT const& normals)
    : _vertices(vertices), _faces(faces), _normals(normals)
{}

DecoratedCloud::DecoratedCloud(CloudT const& vertices, NormalsT const& normals)
    : _vertices(vertices), _normals(normals)
{}

} //...ns acq
