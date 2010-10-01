FOREACH( myfolder
         engine_seamless
         seamless
         applications
         )

         ADD_SUBDIRECTORY(${myfolder})

ENDFOREACH( myfolder)
