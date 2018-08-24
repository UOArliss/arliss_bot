#include <stdio.h>
#include <math.h>

#define EARTH_RADIUS 6371
#define RAD2DEG(deg) (deg * 180.0 / M_PI)
#define DEG2RAD(deg) (deg * M_PI / 180.0f)

/*haversine to find distance on greate circle*/
double dist(double lat1, double lon1, double lat2, double lon2)
{
	double dx, dy, dz;
	lon1 -= lon2;
	lon1 = DEG2RAD(lon1);
	lat1 = DEG2RAD(lat1);
	lat2 = DEG2RAD(lat2);
 
	dz = sin(lat1) - sin(lat2);
	dx = cos(lon1) * cos(lat1) - cos(lat2);
	dy = sin(lon1) * cos(lat1);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * EARTH_RADIUS;
}


/*takes a start lat,lon and dest lat lon returns desired compass bearing 
clockwise*/
double calc_angle(double lat1, double lon1, double lat2, double lon2) {

	lat1 = DEG2RAD(lat1);
	lon1 = DEG2RAD(lon1);
	lat2 = DEG2RAD(lat2);
	lon2 = DEG2RAD(lon2);

    double difflon = (lon2 - lon1);

    double y = sin(difflon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1)* cos(lat2) * cos(difflon);

    double bearing = atan2(y, x);

    bearing = RAD2DEG(bearing);
    bearing = fmod((bearing + 360) , 360);
    /*remove the 360 - to get in counter clockwise*/
    return 360 - bearing;
}


/*distance in km , * 1000 for meters
int main(int argc , char** argv){
	/*first lat/lon eugene , second portland
	double d = dist(44.0521,123.0868,43.7465, 122.4617 );
	double r = calc_angle(44.0521,123.0868,43.7465, 122.4617 );
	printf("%f\n%f",r,d );
}
*/
