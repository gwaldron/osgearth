/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/PagedNode>
#include <osgEarth/Utils>

#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

#define LC "[PagedNode] "

using namespace osgEarth;

namespace
{    
    struct PagedNodePseudoLoader : public osgDB::ReaderWriter
    {
        PagedNodePseudoLoader()
        {
            supportsExtension( "osgearth_pseudo_pagednode", "" );
        }

        const char* className() const
        { // override
            return "PagedNodePseudoLoader";
        }

        ReadResult readNode(const std::string& uri, const Options* options) const
        {
            if ( !acceptsExtension( osgDB::getLowerCaseFileExtension(uri) ) )
                return ReadResult::FILE_NOT_HANDLED;

            osg::ref_ptr<PagedNode> node;
            if (!OptionsData<PagedNode>::lock(options, "osgEarth.PagedNode", node))
            {
                OE_WARN << "Internal error - no PagedNode object in OptionsData\n";
                return ReadResult::ERROR_IN_READING_FILE;
            }

            return node->loadChildren();
        }
    };

    REGISTER_OSGPLUGIN(osgearth_pseudo_pagednode, PagedNodePseudoLoader);
}

PagedNode::PagedNode()
{
    _plod = new osg::PagedLOD;
    addChild(_plod);

    _attachPoint = new osg::Group;

    _plod->addChild( _attachPoint );     
}

void PagedNode::build()
{
    // This is your chance to add something to the attachpoint.
}

void PagedNode::setupPaging()
{
    osg::BoundingSphere bs = getKeyBound();

    _plod->setCenter( bs.center() ); 
    _plod->setRadius( bs.radius() );

    if ( hasChildren() )
    {    

        // Now setup a filename on the PagedLOD that will load all of the children of this node.
        _plod->setFileName(1, ".osgearth_pseudo_pagednode");
      
        // assemble data to pass to the pseudoloader
        osgDB::Options* options = new osgDB::Options();
        OptionsData<PagedNode>::set(options, "osgEarth.PagedNode", this);
        _plod->setDatabaseOptions( options );

        double rangeFactor = 6.0;

        // Setup the min and max ranges.
        float minRange = (float)(bs.radius() * rangeFactor);
        bool additive = false;

        if (!additive)
        {
            // Replace mode, the parent is replaced by its children.
            _plod->setRange( 0, minRange, FLT_MAX );
            _plod->setRange( 1, 0, minRange );
        }
        else
        {
            // Additive, the parent remains and new data is added
            _plod->setRange( 0, 0, FLT_MAX );
            _plod->setRange( 1, 0, minRange );
        }
    }
    else
    {
        // no children, so max out the visibility range.
        _plod->setRange( 0, 0, FLT_MAX );
    }   
}

osg::BoundingSphere PagedNode::getKeyBound() const
{
    return osg::BoundingSphere();
}

bool PagedNode::hasChildren() const
{
    return true;
}

osg::Node* PagedNode::loadChildren()
{
    return 0;
}
