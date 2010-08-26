#include <seamless/Geographic>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osg/Math>
#include <osg/Texture2D>

#include <osgEarth/ImageUtils>
#include <osgEarth/Notify>

#include <seamless/GeoPatch>

namespace seamless
{
using namespace std;
using namespace osg;
using namespace osgEarth;

// A fictional value to use as the level 0 edge length. This value is
// divided by 2 at each LOD. It is used instead of the real edge
// length because the patches are not square.
//
// sqrt(earth_surface_area / 6)
const double edgeLength0 = sqrt(
    4.0 * PI * WGS_84_RADIUS_EQUATOR * WGS_84_RADIUS_EQUATOR / 6.0);

// Hard-wire the patch resolution and screen-space polygon size.
Geographic::Geographic(Map* map)
    : PatchSet(64, new GeographicOptions), _map(map), _profile(new EulerProfile),
      _eModel(new EllipsoidModel)
{
    setPrecisionFactor(8);
    const MapLayerList& heightList = _map->getHeightFieldMapLayers();
    {
        int maxLevel = 0;
        Threading::ScopedReadLock lock(_map->getMapDataMutex());
        for (MapLayerList::const_iterator itr = heightList.begin(),
                 end = heightList.end();
             itr != end;
             ++itr)
            if ((*itr)->maxLevel().isSet()
                && (*itr)->maxLevel().get() > maxLevel)
                maxLevel = (*itr)->maxLevel().get();
        if (maxLevel > 0)
            setMaxLevel(maxLevel);
    }
}

Geographic::Geographic(const Geographic& rhs, const osg::CopyOp& copyop)
    : PatchSet(rhs, copyop), _map(static_cast<Map*>(copyop(rhs._map.get()))),
      _profile(static_cast<EulerProfile*>(copyop(rhs._profile.get()))),
      _eModel(static_cast<EllipsoidModel*>(copyop(rhs._eModel.get())))
{
}

Node* Geographic::createPatchSetGraph(const std::string& filename)
{
    CoordinateSystemNode* csn = new CoordinateSystemNode;
    // Should these values come from the map profile?
    csn->setCoordinateSystem("EPSG:4326");
    csn->setFormat("WKT");
    csn->setEllipsoidModel(_eModel.get());
    for (int face = 0; face < 6; ++face)
    {
        double x = 0.0, y = 0.0;
        euler::faceToCube(x, y, face);
        GeographicOptions* goptions = static_cast<GeographicOptions*>(
            osg::clone(getPatchOptionsPrototype()));
        goptions->setPatchSet(this);
        goptions->setTileKey(_profile->createTileKey(x, y, 0));
        Node* node = createPatchGroup("foobar.tengpatch", goptions);
        csn->addChild(node);
    }
    return csn;
}

namespace
{

GeoHeightField*
mergeHeightFields(const GeoExtent& targetExtent, const GeoHeightFieldList& hfs)
{
    if (hfs.size() != 4)
    {
        OE_FATAL << "mergeHeightFields expected 4 height fields\n";
        return 0;
    }
    // List is in tile subkey quadrant order.
    // Assume the height fields all have the same dimensions
    int targetCols = hfs[0]->getHeightField()->getNumColumns() * 2 - 1;
    int targetRows = hfs[0]->getHeightField()->getNumRows() * 2 - 1;
    HeightField* targethf = new HeightField;
    targethf->allocate(targetCols, targetRows);
    GeoHeightField* geo = new GeoHeightField(targethf, targetExtent, 0);
    for (int i = 0; i < 4; ++i)
    {
        HeightField* src = hfs[i]->getHeightField();
        int targetColumn
            = floor((hfs[i]->getGeoExtent().xMin() - targetExtent.xMin())
                    / targetExtent.width() * (targetCols - 1) + .5);
        int targetRow
            = floor((hfs[i]->getGeoExtent().yMin() - targetExtent.yMin())
                    / targetExtent.height() * (targetRows - 1) + .5);
        for (int sj = 0, tj = targetRow;
             sj < src->getNumRows() && tj < targetRows;
             ++sj, ++tj)
        {
            for (int si = 0, ti = targetColumn;
             si < src->getNumColumns() && ti < targetCols;
             ++si, ++ti)
                targethf->setHeight(ti, tj, src->getHeight(si, sj));
        }
    }
    return geo;
}

GeoImage*
mergeImages(const GeoExtent& targetExtent, const GeoImageList& imgs)
{
    Image* targetImage = new Image;
    const Image* proto = imgs[0]->getImage();
    targetImage->setInternalTextureFormat(proto->getInternalTextureFormat());
    int numRows = proto->s() * 2;
    int numCols = proto->t() * 2;
    targetImage->allocateImage(numRows, numCols, proto->r(),
                               proto->getPixelFormat(), proto->getDataType(),
                               proto->getPacking());
    for (GeoImageList::const_iterator itr = imgs.begin(),
             end = imgs.end();
         itr != end;
         ++itr)
    {
        const GeoExtent& srcExtent = (*itr)->getExtent();
        int dstx
            = floor((srcExtent.xMin() - targetExtent.xMin()) / targetExtent.width()
                    * numCols + .5);
        int dsty
            = floor((srcExtent.yMin() - targetExtent.yMin()) / targetExtent.height()
                    * numRows + .5);
        ImageUtils::copyAsSubImage((*itr)->getImage(), targetImage,
                                   dstx, dsty);
    }
    return new GeoImage(targetImage, targetExtent);
}
}
// Create the geometry for a patch
MatrixTransform* Geographic::createPatchAux(const TileKey* key,
                                            const GeoHeightField* hf)
{
    GeoPatch* patch = new GeoPatch;
    patch->setEdgeLength(edgeLength0 / pow(2.0, key->getLevelOfDetail()));
    patch->setPatchSet(this);
    const GeoExtent& patchExtent = key->getGeoExtent();
    double centx, centy;
    patchExtent.getCentroid(centx, centy);
    Vec3d patchCenter = toModel(centx, centy, 0);
    Matrixd patchMat = Matrixd::translate(patchCenter);
    const SpatialReference* srs = key->getProfile()->getSRS();
    const SpatialReference* geoSrs = srs->getGeographicSRS();
    // Populate cell
    ref_ptr<Patch::Data> data = new Patch::Data;
    int patchDim = _resolution + 1;
    Vec3Array* verts = new Vec3Array(patchDim * patchDim);
    Vec3Array* normals = new Vec3Array(patchDim * patchDim);
    Vec2Array* texCoords = new Vec2Array(patchDim * patchDim);
    double xInc = (patchExtent.xMax() - patchExtent.xMin()) / _resolution;
    double yInc = (patchExtent.yMax() - patchExtent.yMin()) / _resolution;
    for (int j = 0; j < patchDim; ++j)
    {
        for (int i = 0; i < patchDim; i++)
        {
            Vec2d cubeCoord(patchExtent.xMin() + i * xInc,
                            patchExtent.yMin() + j * yInc);
            double lon, lat;
            srs->transform(cubeCoord.x(), cubeCoord.y(), geoSrs, lon, lat);
            float elevation;

            bool found = hf->getElevation(srs, cubeCoord.x(), cubeCoord.y(),
                                          INTERP_BILINEAR, 0, elevation);
            // Into ec coordinates
            if (!found)
            {
                OE_WARN << "Couldn't find height sample for cube coordinates "
                        << cubeCoord.x() << ", " << cubeCoord.y()
                        << " (lon lat " << lon << ", " << lat << ")\n";
                continue;
            }
            Vec3d coord;
            _eModel->convertLatLongHeightToXYZ(
                DegreesToRadians(lat), DegreesToRadians(lon), elevation,
                coord.x(), coord.y(), coord.z());
            (*verts)[j * patchDim + i] = coord - patchCenter;
            if (fabs((*verts)[j * patchDim + i].z()) > 6000000)
            {
                OE_WARN << "found huge coordinate.\n";
            }
            (*texCoords)[j * patchDim +i]
                = Vec2(i / static_cast<float>(_resolution),
                       j / static_cast<float>(_resolution));
        }
    }
    // Normals. Average the normals of the triangles around the sample
    // point. We're not following the actual tessallation of the grid.
    for (int j = 0; j < patchDim; ++j)
    {
        for (int i = 0; i < patchDim; i++)
        {
            const Vec3& pt = (*verts)[j * patchDim + i];
            // A cross of points.
            Vec3 delta[4];      // Initialized to zero vectors
            for (int k = 0; k < 2; ++k)
            {
                int gridx = i + 2 * k - 1;
                if (gridx < patchDim && gridx >= 0)
                    delta[2 * k] = (*verts)[j * patchDim + gridx] - pt;
            }
            for (int k = 0; k < 2; ++k)
            {
                int gridy = j + 2 * k - 1;
                if (gridy < patchDim && gridy >= 0)
                    delta[2 * k + 1] = (*verts)[gridy * patchDim + i] - pt;
            }
            Vec3 normal;
            for (int k = 1; k <= 4; ++k)
            {
                int v1 = k - 1, v2 = k % 4;
                // If One or both of the deltas are 0, then the cross
                // product is 0 and won't contribute to the average.
                normal += delta[v1] ^ delta[v2];
            }
            normal.normalize();
            (*normals)[j * patchDim + i] = normal;
        }
    }
    // Construct the patch and its transform.
    data->vertexData.array = verts;
    data->vertexData.binding = Geometry::BIND_PER_VERTEX;
    data->normalData.array = normals;
    data->normalData.binding = Geometry::BIND_PER_VERTEX;
    Vec4Array* colors = new Vec4Array(1);
    (*colors)[0] = Vec4(1.0, 1.0, 1.0, 1.0);
    data->colorData.array = colors;
    data->colorData.binding = Geometry::BIND_OVERALL;
    data->texCoordList
        .push_back(Geometry::ArrayData(texCoords, Geometry::BIND_PER_VERTEX));
    patch->setData(data);
    MatrixTransform* result = new MatrixTransform;
    result->addChild(patch);
    result->setMatrix(patchMat);
    return result;
}

namespace
{
// Get a height field from the map, or an empty one if there is no
// data for this tile.
GeoHeightField* getGeoHeightField(Map* map, const TileKey* key, int resolution)
{
    HeightField* hf = map->createHeightField(key, true, INTERP_BILINEAR);
    if  (!hf)
        hf = key->getProfile()->getVerticalSRS()
            ->createReferenceHeightField(key->getGeoExtent(),
                                         resolution + 1, resolution + 1);
    return new GeoHeightField(hf, key->getGeoExtent(),
                              key->getProfile()->getVerticalSRS());
}
}

Node* Geographic::createPatch(const std::string& filename,
                              PatchOptions* poptions)
{
    GeographicOptions* goptions = static_cast<GeographicOptions*>(poptions);
    const TileKey* patchKey = goptions->getTileKey();
    int face = EulerProfile::getFace(patchKey);
    const GeoExtent& keyExtent = patchKey->getGeoExtent();
    ref_ptr<GeoHeightField> hf;
    ref_ptr<GeoImage> gimage;
    // Split up patch keys that cross the Date Line. The only patches
    // that do are the the equatorial face with center at (-180, 0),
    // and the poles faces.
    bool crossesDateLine = ((face == 2 || face == 4 || face == 5)
                            && keyExtent.xMax() - keyExtent.xMin() > .5);
    if (crossesDateLine)
    {
        GeoHeightFieldList hfs;
        GeoImageList gis;
        for (int child = 0; child < 4; ++child)
        {
            ref_ptr<TileKey> subCubeKey = patchKey->createSubkey(child);
            hfs.push_back(getGeoHeightField(_map, subCubeKey, _resolution));
            if (!_map->getImageMapLayers().empty())
                gis.push_back(_map->getImageMapLayers()[0]
                              ->createImage(subCubeKey));
        }
        hf = mergeHeightFields(patchKey->getGeoExtent(), hfs);
        if (!gis.empty())
            gimage = mergeImages(patchKey->getGeoExtent(), gis);
    }
    else
    {
        hf = getGeoHeightField(_map, patchKey, _resolution);
        if (!_map->getImageMapLayers().empty())
            gimage = _map->getImageMapLayers()[0]->createImage(patchKey);
    }
    MatrixTransform* transform = createPatchAux(patchKey, hf.get());
    if (gimage)
    {
        Texture2D* tex = new Texture2D();
        tex->setImage(gimage->getImage());
        tex->setWrap(Texture::WRAP_S, Texture::CLAMP_TO_EDGE);
        tex->setWrap(Texture::WRAP_T, Texture::CLAMP_TO_EDGE);
        tex->setFilter(Texture::MIN_FILTER, Texture::LINEAR_MIPMAP_LINEAR);
        tex->setFilter(Texture::MAG_FILTER, Texture::LINEAR);
        Patch* patch = dynamic_cast<Patch*>(transform->getChild(0));
        StateSet* ss = patch->getOrCreateStateSet();
        ss->setTextureAttributeAndModes(0, tex, StateAttribute::ON);
    }
    return transform;
}

namespace
{
ClusterCullingCallback*
createClusterCullingCallback(const Matrixd& transform, const Patch* patch,
                             const EllipsoidModel* et)
{
    //This code is a very slightly modified version of the
    //DestinationTile::createClusterCullingCallback in
    //VirtualPlanetBuilder.
    double globe_radius =  et->getRadiusPolar();
    Vec3 center_position(transform.getTrans());
    Vec3 center_normal(center_position);
    center_normal.normalize();

    unsigned int r,c;

    // populate the vertex/normal/texcoord arrays from the grid.

    float min_dot_product = 1.0f;
    float max_cluster_culling_height = 0.0f;
    float max_cluster_culling_radius = 0.0f;
    const Vec3Array* verts = static_cast<const Vec3Array*>(
        patch->getData()->vertexData.array.get());
    for (Vec3Array::const_iterator itr = verts->begin(), end = verts->end();
         itr != end;
         ++itr)
    {
        Vec3d dv = *itr;
        Vec3d v = dv + center_position;
        double lat, lon, height;

        et->convertXYZToLatLongHeight(v.x(), v.y(), v.z(),
                                      lat, lon, height);

        double d = sqrt(dv.x()*dv.x() + dv.y()*dv.y() + dv.z()*dv.z());
        double theta = acos(globe_radius / (globe_radius + fabs(height)));
        double phi = 2.0 * asin (d*0.5 / globe_radius); // d/globe_radius;
        double beta = theta + phi;
        double sb = sin(beta);
        double cb = cos(beta);
        double cutoff = osg::PI_2 - 0.1;

        //log(osg::INFO,"theta="<<theta<<"\tphi="<<phi<<" beta "<<beta);
        if (phi<cutoff && beta<cutoff)
        {
            float local_dot_product = -sb;
            float local_m = globe_radius*( 1.0/ cb - 1.0);
            float local_radius = static_cast<float>(globe_radius * sb / cb); // beta*globe_radius;
            min_dot_product = osg::minimum(min_dot_product, local_dot_product);
            max_cluster_culling_height = osg::maximum(max_cluster_culling_height,local_m);
            max_cluster_culling_radius = osg::maximum(max_cluster_culling_radius,local_radius);
        }
        else
        {
            //log(osg::INFO,"Turning off cluster culling for wrap around tile.");
            return 0;
        }
    }

    osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback;

    ccc->set(center_position + center_normal*max_cluster_culling_height ,
             center_normal,
             min_dot_product,
             max_cluster_culling_radius);

    return ccc;
}
}

Node* Geographic::createPatchGroup(const string& filename,
                                   PatchOptions* poptions)
{
    Node* result = PatchSet::createPatchGroup(filename, poptions);
    PatchGroup* pgroup = dynamic_cast<PatchGroup*>(result);
    // Make a cluster culling callback
    MatrixTransform* transform
        = dynamic_cast<MatrixTransform*>(pgroup->getChild(0));
    Patch* patch = dynamic_cast<Patch*>(transform->getChild(0));
    ClusterCullingCallback* ccc
        = createClusterCullingCallback(transform->getMatrix(), patch,
                                       _eModel.get());
    pgroup->setCullCallback(ccc);
    return pgroup;
}

Vec3d Geographic::toModel(double cubeX, double cubeY, double elevation)
{
    double faceX = cubeX, faceY = cubeY;
    int face;
    euler::cubeToFace(faceX, faceY, face);
    double lat_deg, lon_deg;
    euler::faceCoordsToLatLon(faceX, faceY, face, lat_deg, lon_deg);
    Vec3d result;
    _eModel->convertLatLongHeightToXYZ(
        DegreesToRadians(lat_deg), DegreesToRadians(lon_deg), elevation,
        result.x(), result.y(), result.z());
    return result;
}

Node* Geographic::createChild(const PatchOptions* parentOptions, int childNum)
{
    const GeographicOptions* parentgopt
        = static_cast<const GeographicOptions*>(parentOptions);
    GeographicOptions* goptions = osg::clone(parentgopt);
    goptions->setPatchLevel(parentgopt->getPatchLevel() + 1);
    goptions->setTileKey(parentgopt->getTileKey()->createSubkey(childNum));
    return createPatchGroup("foobies.tengpatch", goptions);

}
}
