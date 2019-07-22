/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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

#ifndef OSGEARTH_MAP_CALLBACK_H
#define OSGEARTH_MAP_CALLBACK_H 1

#include <osgEarth/Common>
#include <osg/ref_ptr>
#include <list>

namespace osgEarth
{
    class MapInfo;
    class Layer;
    struct MapModelChange;

    /**
     * Callback that the Map object uses to notify listeners of map data model changes.
     */
    struct OSGEARTH_EXPORT MapCallback : public osg::Referenced
    {
        //! Invoked when a Map first establishes a valid profile.
        virtual void onMapInfoEstablished( const MapInfo& mapInfo ) { } 

        //! Invoked when a batch operation begins.
        virtual void onBeginUpdate() { }

        //! Invoked when a batch operation ends.
        virtual void onEndUpdate() { }

        //! Invoked whenever a layer is added, removed, or moved from a Map.
        //! The default implementation in turn calls onLayerAdded, onLayerRemoved,
        //! or onLayerMoved, respectively.
        virtual void onMapModelChanged(const MapModelChange& change);

        //! Invoked when a layer is added to a Map.
        virtual void onLayerAdded(Layer* layer, unsigned index) { }

        //! Invoked when a layer is removed from a Map.
        virtual void onLayerRemoved(Layer* layer, unsigned index) { }

        //! Invoked when a layer is re-ordered within a Map.
        virtual void onLayerMoved(Layer* layer, unsigned oldIndex, unsigned newIndex) { }

        //! Invoked when a layer is enabled
        virtual void onLayerEnabled(Layer* layer) { }

        //! Invoked when a layer is disabled
        virtual void onLayerDisabled(Layer* layer) { }

    public:
        /** dtor */
        virtual ~MapCallback() { }

        /**
         * Invokes a callback method for all layers in the specified Map.
         * This is useful if you create a callback for a Map that already has
         * existing Layers in it and you want to "simulate" adding/removing them all.
         */
        void invokeOnLayerAdded(const class Map*);
        void invokeOnLayerRemoved(const class Map*);
    };

    typedef std::list< osg::ref_ptr<MapCallback> > MapCallbackList;
}

#endif // OSGEARTH_MAP_CALLBACK_H
