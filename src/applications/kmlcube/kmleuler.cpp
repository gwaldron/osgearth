#include <iostream>
#include <seamless/Euler>

using namespace std;
using namespace osg;
using namespace seamless;

const char* prolog =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
<kml xmlns=\"http://earth.google.com/kml/2.2\">\n\
<Document>\n\
	<name>KmlFile</name>\n\
	<StyleMap id=\"msn_ylw-pushpin_copy1\">\n\
		<Pair>\n\
			<key>normal</key>\n\
			<styleUrl>#sn_ylw-pushpin_copy1</styleUrl>\n\
		</Pair>\n\
		<Pair>\n\
			<key>highlight</key>\n\
			<styleUrl>#sh_ylw-pushpin_copy1</styleUrl>\n\
		</Pair>\n\
	</StyleMap>\n\
	<Style id=\"sn_ylw-pushpin_copy1\">\n\
		<IconStyle>\n\
			<scale>1.1</scale>\n\
			<Icon>\n\
				<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n\
			</Icon>\n\
			<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>\n\
		</IconStyle>\n\
                <LineStyle><color>ffff00ff</color></LineStyle>\n\
	</Style>\n\
	<Style id=\"sh_ylw-pushpin_copy1\">\n\
		<IconStyle>\n\
			<scale>1.3</scale>\n\
			<Icon>\n\
				<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n\
			</Icon>\n\
			<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>\n\
		</IconStyle>\n\
	</Style>\n\
        <Style id=\"border\">\n\
          <LineStyle><color>ff0000ff</color></LineStyle>\n\
        </Style>\n";

const char* epilog =
    "</Document>\n\
</kml>";

int main(int argc, char** argv)
{
    cout << prolog;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j <= 16; ++j)
        {
            cout << "<Placemark>\n";
            if (j == 0 || j == 16)
                cout << "  <styleUrl>#border</styleUrl>\n";
            else
                cout << "  <styleUrl>#msn_ylw-pushpin_copy1</styleUrl>\n";
            cout << "  <LineString>\n    <tessellate>1</tessellate>\n";
            cout << "    <coordinates>\n";
            for (int k = 0; k <= 16; ++k)
            {
                double lat_deg, lon_deg;
                euler::faceCoordsToLatLon((j * (2.0 / 16.0) - 1.0),
                                           (k * (2.0 / 16.0) - 1.0), i,
                                           lat_deg, lon_deg);
                cout << "        " << lon_deg << "," << lat_deg << ",0 ";
            }
            cout << "    </coordinates>\n  </LineString>\n</Placemark>\n";
        }
        for (int j = 0; j <= 16; ++j)
        {
            cout << "<Placemark>\n";
            if (j == 0 || j == 16)
                cout << "  <styleUrl>#border</styleUrl>\n";
            else
                cout << "  <styleUrl>#msn_ylw-pushpin_copy1</styleUrl>\n";
            cout << "  <LineString>\n    <tessellate>1</tessellate>\n";
            cout << "    <coordinates>\n";
            for (int k = 0; k <= 16; ++k)
            {
                double lat_deg, lon_deg;
                euler::faceCoordsToLatLon((k * (2.0 / 16.0) - 1.0),
                                          (j * (2.0 / 16.0) - 1.0), i,
                                          lat_deg, lon_deg);
                cout << "        " << lon_deg << "," << lat_deg << ",0 ";
            }
            cout << "    </coordinates>\n  </LineString>\n</Placemark>\n";
        }
    }
    cout << epilog;
    return 0;
}
