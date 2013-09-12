About the Project
=================

Introduction
------------

osgEarth_ is a 3D mapping SDK for OpenSceneGraph_ applications.
It's different than traditional terrain engines in an important way:
osgEarth_ does not require you to build a 3D terrain model before you
display it. 
Instead, it will access the raw data sources at application run time and
composite them into a 3D map *on the fly*.
No terrain model is actually stored to disk, though it does use caching
techniques to speed up the rendering of the map.

The goals of osgEarth_ are to:

- Enable the development of 3D geospatial appliations on top of OpenSceneGraph_.
- Make it as easy as possible to visualize terrian models and 3D maps.
- Interoperate with open mapping standards, technologies, and data.


**So if it for me?**

So: does osgEarth replace the need for offline terrain database creation tools? In many cases it does.

Consider using osgEarth_ if you need to:

    - Get a terrain base map up and running quickly and easily
    - Access open-standards map data services like WMS, WCS, or TMS
    - Integrate locally-stored data with web-service-based data
    - Incorporate new geospatial data layers at run-time
    - Run in a "thin-client" environment
    - Deal with data that may change over time
    - Integrate with a commercial data provider


Community Resources
-------------------

Since osgEarth_ is a free open source SDK, the source code is available to
anyone and we welcome and encourage community participation when it comes
to testing, adding features, and fixing bugs.

**Support Forum**

    The best way to interact with the osgEarth team and the user community is
    through the `support forum`_. **Please read** and follow these guidelines for
    using the forum:

    * Sign up for an account and use your real name. You can participate
      anonymously, but using your real name helps build a stronger community
      (and makes it more likely that we will get to your question sooner).
      
    * Limit yourself to *one topic* per post. Asking multiple questions in one
      post makes it too hard to keep track of responses.
      
    * Always include as much supporting information as possible. Post an
      *earth file* or *short code snippet*. Post the output to ``osgearth_version --caps``.
      Post the output to ``gdalinfo`` if you are having trouble with a GeoTIFF
      or other data file. List everything you have tried so far.
      
    * Be patient!

**OSG Forum**

    Since osgEarth_ is built on top of OpenSceneGraph_, many questions we get
    on the message boards are *really* OSG questions. We will still try our
    best to help. But it's worth your while to join the `OSG Mailing List`_ or
    read the `OSG Forum`_ regularly as well.
    
**Social Media**

    * Follow `@pelicanmapping`_ on twitter for updates.
    * Add our `Google+ Page`_ to your circles for gallery shots.

**Professional Services**

    The osgEarth team supports its efforts through professional services. At
    `Pelican Mapping`_ we do custom software development and integration work
    involving osgEarth_ (and geospatial technologies in general). 
    We are based in the US but we work with clients all over the world.
    `Contact us`_ if you need help!

    
License
-------

`Pelican Mapping`_ licenses osgEarth_ under the LGPL_ free open source license. 

This means that:

    1. You can link to the osgEarth_ SDK in any commercial or non-commercial
       application free of charge.
       
    2. If you make any changes to osgEarth_ *itself*, you must make those changes
       available as free open source software under the LGPL license. (Typically
       this means contributing your changes back to the project, but it is
       sufficient to host them in a public GitHub clone.)
       
    3. If you redistribute the osgEarth_ *source code* in any form, you must
       include the associated copyright notices and license information
       unaltered and intact.
       
    4. *iOS / static linking exception*: The LGPL requires that anything statically
       linked to an LGPL library (like osgEarth) also be released under the LGPL.
       We grant an exception to the LGPL in this case. If you statically link 
       osgEarth with your proprietary code, you are *NOT* required to release your
       own code under the LGPL.
       
That's it.

    
Maintainers
-----------

`Pelican Mapping`_ maintains osgEarth_. We are located in the Washington, DC area.

Pelican is Glenn_, Jason_, Jeff_, and Paul_.


.. _osgEarth:        http://osgEarth.org
.. _OpenSceneGraph:  http://openscenegraph.org
.. _Pelican Mapping: http://pelicanmapping.com
.. _LGPL:            http://www.gnu.org/copyleft/lesser.html
.. _Glenn:           http://twitter.com/#!/glennwaldron
.. _Jason:           http://twitter.com/#!/jasonbeverage
.. _Jeff:            http://twitter.com/#!/_jeffsmith
.. _Paul:            http://twitter.com/#!/p_levy
.. _@pelicanmapping: https://twitter.com/pelicanmapping
.. _Google+ Page:    https://plus.google.com/b/104014917856468748129/104014917856468748129/posts

.. _support forum:    http://forum.osgearth.osg
.. _OSG Mailing List: http://lists.openscenegraph.org/listinfo.cgi/osg-users-openscenegraph.org
.. _OSG Forum:        http://forum.openscenegraph.org
.. _Contact us:       http://pelicanmapping.com/?page_id=2

