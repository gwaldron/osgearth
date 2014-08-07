
offset = 4.0; // global: offset from road centerline (m)

// process all segments.
function positionAlongSegments() {
								 
	var output = [];
	feature.properties.heading = 0.0;
	
	if (feature.geometry.type == 'LineString') {
		doLineString(feature, feature.geometry, output);                        
	}
	else if (feature.geometry.type == 'MultiLineString') {
		var output = [];
		for(var i=0; i<feature.geometry.coordinates.length; ++i) {
			doLineString(feature, feature.geometry.coordinates[i], output);
		}
	}
	
	if ( output.length > 0 ) {
		feature.geometry.type = 'MultiPoint';
		feature.geometry.coordinates = output;
		feature.save();
	}
}

// process a single line string.
function doLineString(feature, geom, output) {

	for(i=0; i<geom.length-1; ++i) {
	
		var segment = { 
			p0: geom[i], 
			p1: geom[i+1]
		};
		
		// midpoint of segment:
		var midpoint = {
			x: 0.5*(segment.p0[0] + segment.p1[0]),
			y: 0.5*(segment.p0[1] + segment.p1[1]),
			z: segment.p0.length > 2 ? 0.5*(segment.p0[2] + segment.p1[2]) : 0.0
		};
		
		// directional vector (XY plane) of the road segment:
		var vector = {
			x: segment.p1[0] - segment.p0[0],
			y: segment.p1[1] - segment.p0[1]
		};
		
		// normalize the vector:
		var veclen = Math.sqrt(vector.x*vector.x + vector.y*vector.y);
		vector.x /= veclen;
		vector.y /= veclen;
		
		// perpendicular vector:
		var pvector = {
			x: -vector.y,
			y:  vector.x
		};
		
		output.push( [
			midpoint.x + offset*pvector.x,
			midpoint.y + offset*pvector.y,
			midpoint.z
		] );
		
		/* other side (commented out sine we cannot rotate twice)
		output.push( [
			midpoint.x - offset*pvector.x,
			midpoint.y - offset*pvector.y,
			midpoint.z
		] );
		*/

		// rotate the model to face the street. 
		// caveat: only one 'heading' per feature!
		var angle = 90 + Math.atan2(pvector.y, pvector.x)*57.2958;
		if ( !isNaN(angle) ) {
			feature.properties.heading = angle;
		}
	}
}
              