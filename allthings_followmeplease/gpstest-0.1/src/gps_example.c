#include <glib.h>
#include <gps.h>
#include "math.h" // included for isnan
#include "config.h" // auto generated

gboolean get_position(struct gps_data_t *GPS)
{
  gps_query(GPS, "p\n");

	if (isnan(GPS->fix.latitude) && isnan(GPS->fix.longitude)) {
		// we don't have any coordinates
		g_print(PROGNAME ":failed to get coordinates from gpsd\n");
	} else {
		g_print(PROGNAME ":latitude:%f, longitude:%f\n", GPS->fix.latitude, GPS->fix.longitude);
	}

	return TRUE;

}

int main()
{
	GMainLoop *mainloop;
	static struct gps_data_t *GPS;

	GPS = gps_open("127.0.0.1", "2947");

	if (GPS == NULL) {
		g_print("Could not connect to gpsd!\n");
		return 1;
	}

	mainloop = g_main_loop_new(NULL, FALSE);

	// periodically call get_position
	g_timeout_add((READ_FREQ * 1000), (GSourceFunc)get_position, (gpointer)GPS);

	g_main_loop_run(mainloop);

	return 0;

}
