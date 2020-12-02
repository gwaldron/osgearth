Simple Model
=============
This plugin simply loads an external 3D model and optionally places it at
map coordinates.

Example usage::

    <model name ="model" driver="simple">
        <url>../data/red_flag.osg.100,100,100.scale</url>
        <location>-74.018 40.717 10</location>
    </model>
    
Properties:

    :url:       External model to load
    :location:  Map coordinates at which to place the model. SRS is that of
                the containing map.
    :paged:     If true, the model will be paged in when the camera is within the max range of the location.  If false the model is loaded immediately.

Also see:

    ``simple_model.earth`` sample in the repo
