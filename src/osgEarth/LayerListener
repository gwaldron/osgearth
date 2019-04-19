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
#ifndef OSGEARTH_LAYER_LISTENER_H
#define OSGEARTH_LAYER_LISTENER_H 1

#include <osgEarth/Map>
#include <osgEarth/MapCallback>

namespace osgEarth
{
    /**
     * A helper class for when you need to know when a named layer is added
     * or removed from the Map. Use it like this:
     *
     *   LayerListener<MyClass> L;
     *   ...
     *   L.listen(map, "layerName", targetObject, &MyClass::setLayer);
     *
     * In this example, targetObject is an instance of MyClass, and setLayer
     * is the member function of MyClass to call whenever layer "layerName"
     * arrives in or departs from the Map. The "setLayer" function signature
     * is
     * 
     *   bool setLayer(Layer*);
     *
     * When "layerName" arrives in the map, the listener will call 
     * 
     *   targetObject->setLayer(layer);
     *
     * And it will call the same method with NULL when the layer departs the map.
     */
    template<typename LISTENER, typename LAYERTYPE>
    class LayerListener
    {
    public:
        typedef void(LISTENER::*Function)(LAYERTYPE*);

        struct Entry {
            Entry() { }
            osg::observer_ptr<const Map> _map;
            std::string _layerName;
            LISTENER* _object;
            Function _function;
            osg::ref_ptr<MapCallback> _callback;
            LayerListener<LISTENER,LAYERTYPE>* _listener;
        };

        struct Callback : public MapCallback {
            Entry* _entry;
            Callback(Entry* entry) : _entry(entry) { }
            void onLayerAdded(Layer* layer, unsigned index) {
                LAYERTYPE* layerCast = dynamic_cast<LAYERTYPE*>(layer);
                if (layerCast)
                    _entry->_listener->onLayerAdded(_entry, layerCast, index);
            }
            void onLayerRemoved(Layer* layer, unsigned index) {
                _entry->_listener->onLayerRemoved(_entry, layer, index);
            }
        };

        //! Listens for a layer of type LAYERTYPE and name "layerName" to arrive.
        void listen(const Map* map, const std::string& layerName, LISTENER* object, Function function) {
            if (map) {
                _entries.push_back(Entry());
                Entry& e = _entries.back();
                e._map = map;
                e._layerName = layerName;
                e._object = object;
                e._function = function;
                e._callback = new Callback(&e);
                e._listener = this;
                map->addMapCallback(e._callback.get());

                // See if it's already there
                LAYERTYPE* layer = map->getLayerByName<LAYERTYPE>(layerName);
                if (layer)
                    onLayerAdded(&e, layer, map->getIndexOfLayer(layer));
            }
        }

        //! Listens for the first layer of type LAYERTYPE to arrive (regardles of name)
        void listen(const Map* map, LISTENER* object, Function function) {
            if (map) {
                _entries.push_back(Entry());
                Entry& e = _entries.back();
                e._map = map;
                e._object = object;
                e._function = function;
                e._callback = new Callback(&e);
                e._listener = this;
                map->addMapCallback(e._callback.get());

                // See if it's already there
                LAYERTYPE* layer = map->getLayer<LAYERTYPE>();
                if (layer)
                    onLayerAdded(&e, layer, map->getIndexOfLayer(layer));
            }
        }

        void clear() {
            for (typename std::vector<Entry>::iterator i = _entries.begin(); i != _entries.end(); ++i) {
                if (i->_callback.valid()) {
                    osg::ref_ptr<const Map> map;
                    if (i->_map.lock(map)) {
                        map->removeMapCallback(i->_callback.get());
                    }
                }
            }
            _entries.clear();
        }

        typename std::vector<Entry> _entries;

        ~LayerListener() {
            clear();
        }

        void onLayerAdded(Entry* entry, LAYERTYPE* layer, unsigned index) {
            if (entry->_layerName.empty() || entry->_layerName == layer->getName())
                ((*entry->_object).*(entry->_function))(layer);
        }

        void onLayerRemoved(Entry* entry, Layer* layer, unsigned index) {
            if (entry->_layerName.empty() || entry->_layerName == layer->getName())
                ((*entry->_object).*(entry->_function))(0L);
        }
    };
}

#endif // OSGEARTH_LAYER_LISTENER_H