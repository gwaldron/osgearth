SilverLining Sky
================
Sky model that uses the SilverLining SDK from SunDog Software.

SilverLining SDK requires a valid license code. Without a username and
license code, the SDK will run in "demo mode" and will display a dialog box
every five minutes.

Example usage::

    <map>
        <options>
            <sky driver = "silverlining"
                 hours               = "0.0"
                 ambient             = "0.05"
                 user                = "myname"
                 license_code        = "mycode"
                 clouds              = "false"
                 clouds_max_altitude = "0.0 />

Properties:

	:user:                 User name the SilverLining SDK license
	:license_code:         License code the SilverLining SDK
	:clouds:               Whether to render a local clouds layer
	:clouds_max_altitude:  Maximumum camera altitude at which to start rendering
	                       the clouds layer
   
.. include:: sky_shared.rst
