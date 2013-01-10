About the Project
=================

Introduction
------------

The goals of osgEarth_ are:

- Enable the development of 3D geospatial appliations on top of OpenSceneGraph_.
- Make it as easy as possible to visualize terrian models and 3D maps.
- Interoperate with open mapping standards, technologies, and data.

What is osgEarth?
-----------------

osgEarth_ is a 3D mapping SDK for OpenSceneGraph_ applications.
It's different than traditional terrain engines in an important way:
osgEarth_ does not require you to build a 3D terrain model before you
display it. 
Instead, it will access the raw data sources at application run time and
composite them into a 3D map *on the fly*.
No terrain model is actually stored to disk, though it does use caching
techniques to speed up the rendering of the map.

Is it for you?
--------------

So: does osgEarth replace the need for offline terrain database creation tools? In many cases it does, but as usual .... *it depends*.

Consider using osgEarth_ if you need to:

- Get a terrain base map up and running quickly and easily
- Access open-standards map data services like WMS, WCS, or TMS
- Integrate locally-stored data with web-service-based data
- Incorporate new geospatial data layers at run-time
- Run in a "thin-client" environment
- Deal with data that may change over time
- Integrate with a commercial data provider

License
-------

`Pelican Mapping`_ licenses osgEarth_ under the LGPL_ free open source license. 

This means that:

    1. You can link to the osgEarth_ SDK in any commercial or non-commercial
       application free of charge.
       
    2. If you make any changes to osgEarth_ *itself*, you must make those changes
       available as free open source software under the LGPL license. (Typically
       this means contributing your changes back to the project, but it is
       sufficient to host them in a GitHub clone.)
       
    3. If you redistribute the osgEarth_ *source code* in any form, you must
       distribute the associated copyright notices and license information
       intact.

Maintainers
-----------

The copyright holder for osgEarth_ is `Pelican Mapping`_ Inc.
We are located in the Washington, DC area.

Pelican is Glenn_, Jason_, Jeff_, and Paul_.


.. _osgEarth:        http://osgEarth.org
.. _Pelican Mapping: http://pelicanmapping.com
.. _LGPL:            http://www.gnu.org/copyleft/lesser.html
.. _Glenn:           http://twitter.com/#!/glennwaldron
.. _Jason:           http://twitter.com/#!/jasonbeverage
.. _Jeff:            http://twitter.com/#!/_jeffsmith
.. _Paul:            http://twitter.com/#!/p_levy
