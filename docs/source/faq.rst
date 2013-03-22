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
			

Data
----
			
Features & Symbology
--------------------

Annotations
-----------

Interoperability
----------------

Performance
-----------

Community
---------

**What is the "best practice" for using GitHub?**

	The best way to work with the osgEarth repository is to make your own clone on GitHub
	and to work from that clone. Why not work directly against the main repository? You
	can, but if you need to make changes, bug fixes, etc., you will need your own clone
	in order to issue Pull Requests.
	
		# Create your own GitHub account and log in.
		# Clone the osgEarth repo.
		# Work from your clone. Update it from the main repository peridocially.
	
**How do I submit changes to osgEarth?**

    We accept contributions and bug fixes through GitHub's *pull request* mechanism.
	First you need your own GitHub account and a clone of the repo (see above). Next,
	follow these guidelines:
	
		# Create a *branch* in which to make your changes.
		# Make the change.
		# Issue a *pull request* against the main osgEarth repository.
		# We will review the *PR* for inclusion.

	If we opt NOT to include your submission, you can still keep it in your cloned
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
	LGPL does not support static linking, but we make an exception in cases like
	this.
