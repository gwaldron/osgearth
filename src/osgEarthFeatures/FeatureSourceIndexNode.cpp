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
#include <osgDB/WriteFile>
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

        "in uint oe_fid_attr;\n"
        "out vec4 oe_pick_encoded_fid;\n"
        "void oe_pick_vertex(inout vec4 vertex) \n"
        "{\n"
        "    float b0 = float((oe_fid_attr & uint(0xff000000)) >> 24)/255.0; \n"
        "    float b1 = float((oe_fid_attr & uint(0x00ff0000)) >> 16)/255.0; \n"
        "    float b2 = float((oe_fid_attr & uint(0x0000ff00)) >>  8)/255.0; \n"
        "    float b3 = float((oe_fid_attr & uint(0x000000ff)) >>  0)/255.0; \n"
        "    oe_pick_encoded_fid = vec4(b0, b1, b2, b3); \n"
        "} \n";

    const char* pickFragment =
        "in vec4 oe_pick_encoded_fid;\n"
        "void oe_pick_fragment(inout vec4 color)\n"
        "{\n"
        "    gl_FragColor = oe_pick_encoded_fid; \n"
        "}\n";
}

FeaturePicker::FeaturePicker(int size)
{
    initialize( size );
}

FeaturePicker::~FeaturePicker()
{
    if ( _rtt.valid() )
    {
        while( _rtt->getNumParents() > 0 )
        {
            _rtt->getParent(0)->removeChild( _rtt.get() );
        }
    }
}

void
FeaturePicker::initialize(int rttSize)
{
    _rttSize = std::max(rttSize, 4);

    _pickInProgress = false;
    
    _buffer = 2;

    _image = new osg::Image();
    _image->allocateImage(_rttSize, _rttSize, 1, GL_RGBA, GL_UNSIGNED_BYTE);    
    
    _rtt = new osg::Camera();
    _rtt->setClearColor( osg::Vec4(0,0,0,0) );
    _rtt->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT ); 
    _rtt->setViewport( 0, 0, _rttSize, _rttSize );
    _rtt->setRenderOrder( osg::Camera::PRE_RENDER );
    _rtt->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    _rtt->attach( osg::Camera::COLOR_BUFFER0, _image.get() );
    
    osg::StateSet* rttSS = _rtt->getOrCreateStateSet();

    osg::StateAttribute::GLModeValue disable = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;

    rttSS->setMode(GL_BLEND,     disable );    
    rttSS->setMode(GL_LIGHTING,  disable );
    rttSS->setMode(GL_CULL_FACE, disable );

    VirtualProgram* vp = VirtualProgram::getOrCreate( rttSS );
    vp->setFunction( "oe_pick_vertex",   pickVertex,   ShaderComp::LOCATION_VERTEX_MODEL );
    vp->setFunction( "oe_pick_fragment", pickFragment, ShaderComp::LOCATION_FRAGMENT_OUTPUT );
    vp->addBindAttribLocation( "oe_fid_attr", FeatureSourceIndexNode::IndexAttrLocation );
}

osg::Texture2D*
FeaturePicker::getTexture()
{
    if ( !_tex.valid() )
    {
        _tex = new osg::Texture2D(_image.get());
        _tex->setTextureSize(_image->s(), _image->t());
        _tex->setUnRefImageDataAfterApply(false);
        _tex->setFilter(_tex->MIN_FILTER, _tex->NEAREST);
        _tex->setFilter(_tex->MAG_FILTER, _tex->NEAREST);
    }
    return _tex.get();
}

bool
FeaturePicker::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if ( _pickInProgress && ea.getEventType() == ea.FRAME )
    {
        checkForPickResults();
        _pickInProgress = false;
    }
    else if ( !_pickInProgress && _callback.valid() && _callback->accept(ea, aa) )
    {
        _pickInProgress = pick(ea, aa);
        if ( _pickInProgress )
            aa.requestRedraw();
    }
    return false;
}

bool
FeaturePicker::pick(osg::View* view, float nx, float ny)
{
    if ( !view )
        return false;

    osg::Camera* cam = view->getCamera();
    if ( !cam )
        return false;

    const osg::Viewport* vp = cam->getViewport();
    if ( !vp )
        return false;

    if ( _rtt->getNumParents() == 0 )
    {
        cam->addChild( _rtt.get() );
    }

    float u = 0.5f*(nx + 1.0f);
    float v = 0.5f*(ny + 1.0f);

    double L, R, B, T, N, F;
    cam->getProjectionMatrix().getFrustum(L, R, B, T, N, F);

    double X = L + u*(R-L);
    double Y = B + v*(T-B);
    double U = (R-L)/vp->width();
    double V = (T-B)/vp->height();

    L = X - U*0.5*double(_image->s());
    R = X + U*0.5*double(_image->s());
    B = Y - V*0.5*double(_image->t());
    T = Y + V*0.5*double(_image->t());
    
    _rtt->setProjectionMatrix( osg::Matrix::frustum(L, R, B, T, N, F) );

    _rtt->setProjectionMatrix( cam->getProjectionMatrix() );
    _rtt->setViewMatrix( cam->getViewMatrix() );
    _pickU = u;
    _pickV = v;
    
    return true;
}

bool
FeaturePicker::pick(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter&      aa)
{
    if ( !_callback.valid() )
        return false;

    return pick(aa.asView(), ea.getXnormalized(), ea.getYnormalized());
}

namespace
{
    // Iterates through the pixels in a grid, starting at u,v [0..1] and spiraling out.
    // http://stackoverflow.com/a/14010215/4218920
    struct SpiralIterator
    {
        unsigned _ring;
        unsigned _maxRing;
        unsigned _leg;
        int      _x, _y;
        int      _w, _h;
        int      _offsetX, _offsetY;
        unsigned _count;

        SpiralIterator(int w, int h, int maxDist, float u, float v) : 
            _w(w), _h(h), _maxRing(maxDist), _count(0), _ring(1), _leg(0), _x(0), _y(0)
        {
            _offsetX = (int)(u * (float)w);
            _offsetY = (int)(v * (float)h);
        }

        bool next()
        {
            if ( _count++ == 0 )
                return true;

            do {
                switch(_leg) {
                case 0: ++_x; if (  _x == _ring ) ++_leg; break;
                case 1: ++_y; if (  _y == _ring ) ++_leg; break;
                case 2: --_x; if ( -_x == _ring ) ++_leg; break;
                case 3: --_y; if ( -_y == _ring ) { _leg = 0; ++_ring; } break;
                }
            }
            while(_ring <= _maxRing && (_x+_offsetX < 0 || _x+_offsetX >= _w || _y+_offsetY < 0 || _y+_offsetY >= _h));

            return _ring <= _maxRing;
        }

        int s() const { return _x+_offsetX; }

        int t() const { return _y+_offsetY; }
    };
}

void
FeaturePicker::checkForPickResults()
{    
    //osgDB::writeImageFile( *_image.get(), "out.png" );

    // decode the results
    ImageUtils::PixelReader read(_image.get());

    SpiralIterator iter(_image->s(), _image->t(), std::max(_buffer,1), _pickU, _pickV);
    while(iter.next())
    {
        osg::Vec4f value = read(iter.s(), iter.t());

        unsigned fidPlusOne =
            ((unsigned)(value.r()*255.0) << 24) +
            ((unsigned)(value.g()*255.0) << 16) +
            ((unsigned)(value.b()*255.0) <<  8) +
            ((unsigned)(value.a()*255.0));

        if ( fidPlusOne > 0 )
        {
            _callback->onHit( FeatureID(fidPlusOne-1u) );
            return;
        }
    }

    _callback->onMiss();
}

bool
FeaturePicker::addChild( osg::Node* child )
{
    return _rtt->addChild( child );
}

bool
FeaturePicker::insertChild( unsigned i, osg::Node* child )
{
    return _rtt->insertChild( i, child );
}

bool
FeaturePicker::removeChild( osg::Node* child )
{
    return _rtt->removeChild( child );
}

bool
FeaturePicker::replaceChild( osg::Node* oldChild, osg::Node* newChild )
{
    return _rtt->replaceChild( oldChild, newChild );
}

//-----------------------------------------------------------------------------


FeatureSourceIndexOptions::FeatureSourceIndexOptions(const Config& conf) :
_enabled      ( true ),
_embedFeatures( false )
{
    conf.getIfSet( "enabled",        _enabled );
    conf.getIfSet( "embed_features", _embedFeatures );
}

Config
FeatureSourceIndexOptions::getConfig() const
{
    Config conf("feature_indexing");
    conf.addIfSet( "enabled",        _enabled );
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

    // The tag is actually FeatureID + 1, to preserve "0" as an "empty" value.
    int tag = feature->getFID() + 1;
    ids->assign( geom->getVertexArray()->getNumElements(), tag );

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
