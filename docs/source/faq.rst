FAQ
===

Miscellaneous
-------------

**How do make the globe transparent so I can see underground?**

	By default, the globe will be opaque white when there are no image layers, or when all the image
	layers have their opacities set to zero. To make the underlying globe transparent, you need to 
	enable *terrain blending*, like so::
	
		<map>
		  <options>
		    <terrain blending="true" ...
			

Other Terrain Technologies
--------------------------

**Does osgEarth work with VirtualPlanetBuilder?**

	VirtualPlanetBuilder_ (VPB) is a command-line terrain generation tool. Before osgEarth
	came along, VPB	was probably the most-used open source tool for building terrains for
	OSG appliations. We	mention is here because many people ask questions about loading 
	VPB models or transitioning from VPB to osgEarth.
	
	osgEarth differs from VPB in that:
	
	* VPB builds static terrain models and saves them to disk. osgEarth generates terrain on
	  demand as your application runs; you do not (and cannot) save a model to disk.
	* Changing a VPB terrain generally requires that you rebuild the model. osgEarth does not
	  require a preprocessing step since it builds the terrain at run time.
	* osgEarth and VPB both use *GDAL* to read many types of imagery and elevation data from
	  the local file system. osgEarth also supports network-based data sources through its
	  plug-in framework.

	osgEarth has a *VPB driver* for "scraping" elevation and imagery tiles from a VPB model.
	See the ``vpb_earth_bayarea.earth`` example in the repo for usage.
	
	**Please Note** that this driver only exists as a **last resort** for people that have a VPB
	model but no longer have access to the source data from which it was built. If at all
	possible you should feed your source data directly into osgEarth instead of using the VPB
	driver.
	
**Can osgEarth load TerraPage or MetaFlight?**

	osgEarth cannot natively load TerraPage (TXP) or MetaFlight. However, osgEarth does have a
	"bring your own terrain" plugin that allows you to load an external model and use it as your
	terrain. The caveat is that since osgEarth doesn't know anything about your terrain model, you
	will not be able to use some of the features of osgEarth (like being able to add or remove layers).
	
	For usage formation, please refer to the ``byo.earth`` example in the repo.

.. _VirtualPlanetBuilder:	http://www.openscenegraph.com/index.php/documentation/tools/virtual-planet-builder


Community
---------

**What is the "best practice" for using GitHub?**

	The best way to work with the osgEarth repository is to make your own clone on GitHub
	and to work from that clone. Why not work directly against the main repository? You
	can, but if you need to make changes, bug fixes, etc., you will need your own clone
	in order to issue Pull Requests.
	
	1. Create your own GitHub account and log in.
	2. Clone the osgEarth repo.
	3. Work from your clone. Update it from the main repository peridocially.
	
**How do I submit changes to osgEarth?**

	We accept contributions and bug fixes through GitHub's *pull request* mechanism.

	First you need your own GitHub account and a clone of the repo (see above). Next,
	follow these guidelines:
	
	1. Create a *branch* in which to make your changes.
	2. Make the change.
	3. Issue a *pull request* against the main osgEarth repository.
	4. We will review the *PR* for inclusion.

	If we decide NOT to include your submission, you can still keep it in your cloned
	repository and use it yourself. Doing so maintains compliance with the osgEarth
	license since your changes are still available to the public - even if they are
	not merged into the master repository.
	
Licensing
---------

**Can I use osgEarth in a commercial product?**

	Yes. The license permits use in a commercial product. The only requirement is that
	any changes you make to the actual osgEarth library *itself* be made available
	under the same license as osgEarth. You do *not* need to make other parts of your
	application public.
	
**Can I use osgEarth in an iOS app?**

	Yes. Apple's policy requires only statically linked libraries. Technically, the
	LGPL does not support static linking, but we grant an exception in this case.
