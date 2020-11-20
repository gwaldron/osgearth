Simple Sky
==========
Sky model that implements atmospheric scattering and lighting according to the
Sam O'Neil GPU Gems article.

Example usage::

    <map>
        <options>
            <sky driver               = "simple"
			     hours                = "0.0"
                 ambient              = "0.05"
				 atmospheric_lighting = "true" 
				 exposure             = "3.0"  />

Properties:

    :atmospheric_lighting: Whether to apply the atmospheric scattering model to the scene
	                       under the Sky node. If you set this to false, you will get 
						   basic Phong lighting instead.
    :exposure:             Exposure level to apply to the scattering model, which simulates
	                       the wash-out effect of viewing terrain through the atmosphere.
   
.. include:: sky_shared.rst
