/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarth/ImageUtils>
#include <osgEarth/VirtualProgram>
#include <osg/MatrixTransform>
#include <osgViewer/View>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[FeatureSourceIndexNode] "

// for testing:
//#undef  OE_DEBUG
//#define OE_DEBUG OE_INFO

namespace
{
    const char* pickVertex =
        "#version 130\n"
        "in int oe_fid_attr;\n"
        "out vec4 oe_pick_fid;\n"
        "void oe_pick_vertex(inout vec4 vertex) {\n"
        "    int b3 = oe_fid_attr >> 24; \n"
        "    int b2 = (oe_fid_attr >> 16) & 0xff; \n"
        "    int b1 = (oe_fid_attr >> 8) & 0xff; \n"
        "    int b0 = oe_fid_attr & 0xff; \n"
        "    oe_pick_fid = vec4( float(b3)/255.0, float(b2)/255.0, float(b2)/255.0, float(b0)/255.0 ); \n"
        "} \n";

    const char* pickFragment =
        "in vec4 oe_pick_fid;\n"
        "void oe_pick_fragment(inout vec4 color) {\n"
        "    color = oe_pick_fid; \n"
        "}\n";

    struct CB : public osg::NodeCallback
    {
        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            OE_WARN << "Shmoo.\n";
            traverse(node, nv);
        }
    };
}


FeaturePicker::FeaturePicker()
{
    _pickInProgress = false;

    unsigned size = 1;

    _image = new osg::Image();
    _image->allocateImage(size, size, 1, GL_RGBA, GL_UNSIGNED_BYTE);

    _camera = new osg::Camera();
    _camera->setClearColor( osg::Vec4(0,0,0,0) );
    _camera->setReferenceFrame( osg::Camera::RELATIVE_RF );
    _camera->setViewport( 0, 0, size, size );
    _camera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    _camera->setRenderOrder( osg::Camera::PRE_RENDER );
    _camera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    _camera->setImplicitBufferAttachmentMask(0, 0);
    _camera->attach( osg::Camera::COLOR_BUFFER0, _image.get() );
    _camera->setCullingActive(false);

    //_camera->setCullCallback( new CB() );

    VirtualProgram* vp = VirtualProgram::getOrCreate(_camera->getOrCreateStateSet());
    vp->setFunction( "oe_pick_vertex",   pickVertex,   ShaderComp::LOCATION_VERTEX_MODEL );
    vp->setFunction( "oe_pick_fragment", pickFragment, ShaderComp::LOCATION_FRAGMENT_LIGHTING, FLT_MAX );
    vp->addBindAttribLocation( "oe_fid_attr", FeatureSourceIndexNode::IndexAttrLocation );
}

void
FeaturePicker::setPickGraph(osg::Node* graph)
{
    _camera->removeChildren(0, _camera->getNumChildren());

    _graph = graph;

    if ( _graph.valid() )
    {
        _camera->addChild( _graph.get() );
    }
}

bool
FeaturePicker::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    //OE_WARN << "PIP = " << _pickInProgress << "\n";

    if ( _pickInProgress && ea.getEventType() == ea.FRAME )
    {
        _pickInProgress = false;
        checkForPickResults();
    }
    else if ( !_pickInProgress && ea.getEventType() == ea.PUSH )
    {
        pick(ea, aa);
    }
    return false;
}

void
FeaturePicker::pick(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter&      aa)
{
    if ( !_graph.valid() || !_callback.valid() )
        return;
    
    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if ( !view )
        return;

    osg::Camera* cam = view->getCamera();
    if ( !cam )
        return;

    const osg::Viewport* vp = view->getCamera()->getViewport();
    if ( !vp )
        return;

    //_camera->setViewport( new osg::Viewport(*vp) ); //(int)ea.getX(), (int)ea.getY(), 1, 1 );
    _camera->setViewport( (int)ea.getX(), (int)ea.getY(), 1, 1 );

    _pickInProgress = true;
    aa.requestRedraw();
    
    if ( _camera->getNumParents() == 0 )
    {
        cam->addChild( _camera.get() );
    }
}

void
FeaturePicker::checkForPickResults()
{
    // first remove the RTT camera:
    //while( _camera->getNumParents() > 0 )
    //{
    //    _camera->getParent(0)->removeChild( _camera.get() );
    //}

    OE_WARN << "CFPR: s=" << _image->s() << ", t=" << _image->t() << "\n";
    
    // decode the results
    std::set<FeatureID> results;    
    ImageUtils::PixelReader read(_image.get());
    for(int s=0; s<_image->s(); ++s)
    {
        for(int t=0; t<_image->t(); ++t)
        {
            osg::Vec4f value = read(s, t);

            FeatureID fid(
                ((unsigned)(value.r()*255.0) << 24) +
                ((unsigned)(value.g()*255.0) << 16) +
                ((unsigned)(value.b()*255.0) <<  8) +
                ((unsigned)(value.a()*255.0)));

            OE_WARN << "r=" << value.r() << ", g=" << value.g() << ", b=" << value.b() << ", a=" << value.a() << "\n";

            results.insert(fid);
        }
    }

    // invoke the callback
    _callback->onPick(results);
}

//-----------------------------------------------------------------------------


FeatureSourceIndexOptions::FeatureSourceIndexOptions(const Config& conf) :
_embedFeatures( false )
{
    conf.getIfSet( "embed_features", _embedFeatures );
}

Config
FeatureSourceIndexOptions::getConfig() const
{
    Config conf("feature_indexing");
    conf.addIfSet( "embed_features", _embedFeatures );
    return conf;
}


//-----------------------------------------------------------------------------

FeatureSourceIndexNode::Collect::Collect(FeatureIDDrawSetMap& index, int idAttrArraySlot) :
osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
_index          ( index ),
_psets          ( 0 ),
_idAttrArraySlot( idAttrArraySlot )
{
    _index.clear();
}

void
FeatureSourceIndexNode::Collect::apply( osg::Node& node )
{
    RefFeatureID* fid = dynamic_cast<RefFeatureID*>( node.getUserData() );
    if ( fid )
    {
        FeatureDrawSet& drawSet = _index[*fid];
        drawSet.nodes().push_back( &node );
    }
    traverse(node);
}

void
FeatureSourceIndexNode::Collect::apply( osg::Geode& geode )
{
    RefFeatureID* fid = dynamic_cast<RefFeatureID*>( geode.getUserData() );
    if ( fid )
    {
        FeatureDrawSet& drawSet = _index[*fid];
        drawSet.nodes().push_back( &geode );
    }
    else
    {
        for( unsigned i = 0; i < geode.getNumDrawables(); ++i )
        {
            osg::Geometry* geom = dynamic_cast<osg::Geometry*>( geode.getDrawable(i) );
            if ( geom )
            {
                osg::IntArray* ids = dynamic_cast<osg::IntArray*>(geom->getVertexAttribArray(_idAttrArraySlot));

                osg::Geometry::PrimitiveSetList& psets = geom->getPrimitiveSetList();
                for( unsigned p = 0; p < psets.size(); ++p )
                {
                    osg::PrimitiveSet* pset = psets[p];

                    // first check for user data:
                    RefFeatureID* fid = dynamic_cast<RefFeatureID*>( pset->getUserData() );
                    if ( fid )
                    {
                        FeatureDrawSet& drawSet = _index[*fid];
                        drawSet.getOrCreateSlice(geom).push_back(pset);
                        _psets++;
                    }

                    // failing that, check for attribution:
                    else if ( ids )
                    {
                        osg::DrawElements* de = pset->getDrawElements();
                        if ( de && de->getNumIndices() > 0 )
                        {
                            std::set<FeatureID> fidsVisitedInThisPSet;

                            for(unsigned i = 0; i < de->getNumIndices(); ++i)
                            {
                                int vi = de->getElement(i);
                                if ( vi < ids->getNumElements() )
                                {
                                    FeatureID fid( (*ids)[vi] );

                                    if ( fidsVisitedInThisPSet.find(fid) == fidsVisitedInThisPSet.end() )
                                    {
                                        FeatureDrawSet& drawSet = _index[fid];
                                        drawSet.getOrCreateSlice(geom).push_back(pset);
                                        fidsVisitedInThisPSet.insert(fid);
                                        _psets++;
                                    }
                                }
                            }
                        }
                        else
                        {
                            OE_WARN << LC << "TODO: try DrawArrays\n";
                        }
                    }
                }
            }
        }
    }

    // NO traverse.
}

//-----------------------------------------------------------------------------

const int FeatureSourceIndexNode::IndexAttrLocation = osg::Drawable::SECONDARY_COLORS;


FeatureSourceIndexNode::FeatureSourceIndexNode(FeatureSource*                   featureSource, 
                                               const FeatureSourceIndexOptions& options) :
_featureSource  ( featureSource ), 
_options        ( options ),
_idAttrArraySlot( IndexAttrLocation )
{
    //nop
}

FeatureSourceIndexNode::~FeatureSourceIndexNode()
{
    //nop
}

// Rebuilds the feature index based on all the tagged primitive sets found in a graph
void
FeatureSourceIndexNode::reindex()
{
    _drawSets.clear();

    Collect c(_drawSets, _idAttrArraySlot);
    this->accept( c );

    OE_DEBUG << LC << "Reindexed; draw sets = " << _drawSets.size() << std::endl;
}

// Tags the vertex array with the specified FeatureID.
void
FeatureSourceIndexNode::tagDrawable(osg::Drawable* drawable, const Feature* feature) const
{
    if ( drawable == 0L )
        return;

    osg::Geometry* geom = drawable->asGeometry();
    if ( !geom )
        return;

    // add a new integer attributer to store the feautre ID per vertex.
    osg::IntArray* ids = new osg::IntArray();
    ids->setPreserveDataType(true);
    geom->setVertexAttribArray    (_idAttrArraySlot, ids);
    geom->setVertexAttribBinding  (_idAttrArraySlot, osg::Geometry::BIND_PER_VERTEX);
    geom->setVertexAttribNormalize(_idAttrArraySlot, false);

    ids->assign( geom->getVertexArray()->getNumElements(), (int)feature->getFID() );

    // optionally save the actual feature object in the index.
    if ( _options.embedFeatures() == true )
    {
        _features[feature->getFID()] = feature;
    }
}

namespace
{
    struct FindAndTagDrawables : public osg::NodeVisitor
    {
        FindAndTagDrawables(const Feature* f, const FeatureSourceIndex* i) : _feature(f), _index(i)
        {
            setTraversalMode(TRAVERSE_ALL_CHILDREN);
            setNodeMaskOverride(~0);
        }

        void apply(osg::Geode& geode)
        {
            for(unsigned i=0; i<geode.getNumDrawables(); ++i)
            {
                osg::Drawable* d = geode.getDrawable(i);
                if ( d )
                    _index->tagDrawable( d, _feature );
            }
            // not traverse necessary
        }

        const FeatureSourceIndex* _index;
        const Feature*            _feature;
    };
}

void
FeatureSourceIndexNode::tagAllDrawables(osg::Node* node, const Feature* feature) const
{
    if ( node && feature )
    {
        FindAndTagDrawables visitor(feature, this);
        node->accept( visitor );
    }
}

void
FeatureSourceIndexNode::tagNode( osg::Node* node, const Feature* feature ) const
{
    node->setUserData( new RefFeatureID(feature->getFID()) );

    if ( _options.embedFeatures() == true )
    {
        _features[feature->getFID()] = feature;
    }
}


bool
FeatureSourceIndexNode::getAllFIDs(std::vector<FeatureID>& output) const
{
    output.reserve( _drawSets.size() );
    output.clear();
    for(FeatureIDDrawSetMap::const_iterator i = _drawSets.begin(); i != _drawSets.end(); ++i )
    {
        output.push_back( i->first );
    }
    return true;
}

bool
FeatureSourceIndexNode::getFID(osg::Drawable* drawable, int vertIndex, FeatureID& output) const
{
    if ( drawable == 0L || vertIndex < 0 )
        return false;

    osg::Geometry* geom = drawable->asGeometry();
    if ( geom == 0L )
        return false;

    osg::IntArray* ids = dynamic_cast<osg::IntArray*>( geom->getVertexAttribArray(_idAttrArraySlot) );
    if ( ids == 0L )
        return false;
    
    output = (*ids)[vertIndex];
    return true;
}

FeatureDrawSet&
FeatureSourceIndexNode::getDrawSet(const FeatureID& fid )
{
    static FeatureDrawSet s_empty;

    FeatureIDDrawSetMap::iterator i = _drawSets.find(fid);
    return i != _drawSets.end() ? i->second : s_empty;
}


bool
FeatureSourceIndexNode::getFeature(const FeatureID& fid, const Feature*& output) const
{
    if ( _options.embedFeatures() == true )
    {
        FeatureMap::const_iterator f = _features.find(fid);

        if(f != _features.end())
        {
            output = f->second.get();
            return output != 0L;
        }
    }
    else if ( _featureSource.valid() && _featureSource->supportsGetFeature() )
    {
        output = _featureSource->getFeature( fid );
        return output != 0L;
    }

    return false;
}
