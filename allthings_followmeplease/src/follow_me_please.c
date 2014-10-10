/** \file followmeplease.c
 *  \brief A small application that lets the ARDRone2 follow the GCS is a GPS is in the Fieldlaptop
 *
 *   Author: OpenUAS
 *   GPL'd
 *   This application receives position information through gpsd and moves the "stay" waypoint
 *   through ivy bus to the AC
 *
 *   Note that at the moment it is very intertwined with the follow me please flightplan.
 *   Feel free to improve and submit  your changes.
 *
 *  For testing GPSFake comes in handy
 *   http://manpages.ubuntu.com/manpages/precise/man1/gpsfake.1.html
 *
 *   In the end we will have one button only with text "Follow me <> Stop following"
 *   TODO and FIXME alre allowed to be done and fixed ;)
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <glib.h>
/*#include <gtk/gtk.h> is what the previous line should read when compiling with pkg-config --cflags --libs gtk+-2.0 */
#include <gtk-2.0/gtk/gtk.h>
#include <unistd.h>

//#include <signal.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/version.h>

#include "gps.h" /* GPS d Library */

#include "nextwp.c" /* remove later*/
//#include "waypoint.h" /* remove later */

/* Interface widget  */
GtkWidget *wndMain;
GtkWidget *btnQuit;
GtkWidget *boxMain;
GtkWidget *lblStatusline;
GtkWidget *btnStartStop ;

char status_str[256];

/* typedef struct Waypoint {
    int ac_id;
    int wp;
    float lat;
    float lon;
    float alt;
} Waypoint;

#endif */ /* _WAYPOINT_H */

float END_LAT = 0.0; //latitude of the "end" waypoint (the fifth waypoint defined in flight plan)
float END_LON = 0.0; //longitude of the "end" waypoint (the fifth waypoint defined in flight plan)
int CONTINUE = 1; //When this variable is set to 0(FALSE) the application will stop moving the waypoint around
float ADD = 0; //just to move the waypoint around for fun

#define MSG_DEST	"ground"
#define MSG_NAME  "FLIGHT_PARAM"
//A special case where MSG_ID="GCS" gets handeled differently by orignal PPRZ GCS
#define MSG_ID		"GCS"
#define TIMEOUT_PERIOD 200

//Temp from generated flightplan
#define WP__CLIMB 2
#define WP_Landingspot 3
#define WP__TDEMERGENCY 4
#define WP_A 5
//TEMP a shadow waypoint with a small offset so it is easier to seee n debug on GCS not under GPSd red dot
#define WP_B 6

/* #define FP_BLOCKS { \
 { "Setting home location" },0 \
 { "Holding point" }, 1\
 { "Takeoff" }, 2\
 { "Follow" },3 \
 { "Land" }, 4\
 { "Emergency" }, 5\
 { "HOME" }, 6\ */

int doFollowMe = 0;  //When this variable is set to FALSE follow_me_please will stop the following

int STAY_WP_INDEX = WP_A;

struct gps_data_t *gpsdata;

Waypoint new_wp;

gboolean verbose;

int arg_ac_id;
char* server;
char* port;
char* ivy_bus;

/* callback associated to "Hello" messages */
void textCallback(IvyClientPtr app, void *data, int argc, char **argv)
{
    const char* arg = (argc < 1) ? "" : argv[0];
    IvySendMsg("Good %s", arg);
}

/* Sets the global variables END_LAT & END_LON to the GPS coordinates
 * of the end waypoint (the fifth waypoint defined in the flight plan).
 */
void set_end(IvyClientPtr app, void *data, int argc, char **argv)
{
    const char *arg = (argc < 1) ? "" : argv[0];

    char *current;
    current = strtok(arg, " ");
    current = strtok(NULL, " ");
    current = strtok(NULL, " ");
    current = strtok(NULL, " ");
    END_LAT = atof(current);
    current = strtok(NULL, " ");
    END_LON = atof(current);
    if (verbose) g_print ("END_LAT=%f. END_LON=%f.\n", END_LAT, END_LON);
    //fprintf(stderr, "END_LAT=%f. END_LON=%f.", END_LAT, END_LON);
}

static void update_gps(struct gps_data_t *gpsdata, char *message, size_t len) {

  static double fix_time = 0;
  double fix_track = 0;
  double fix_speed = 0;
  double fix_altitude = 0;
  double fix_climb = 0;

    if ((isnan(gpsdata->fix.latitude) == 0) &&
        (isnan(gpsdata->fix.longitude) == 0) &&
        (isnan(gpsdata->fix.time) == 0) &&
        (gpsdata->fix.mode >= MODE_2D) &&
        (gpsdata->fix.time != fix_time))
    {
        if (!isnan(gpsdata->fix.track))
            fix_track = gpsdata->fix.track;
        if (!isnan(gpsdata->fix.speed))
            fix_speed = gpsdata->fix.speed;

        if (gpsdata->fix.mode >= MODE_3D)
        {
            if (!isnan(gpsdata->fix.altitude))
                fix_altitude = gpsdata->fix.altitude;
            if (!isnan(gpsdata->fix.climb))
                fix_climb = gpsdata->fix.climb;
        }

        if (verbose)
          g_print ("sending gps info via Ivy: lat %g, lon %g, speed %g, course %g, alt %g, climb %g\n",
                   gpsdata->fix.latitude, gpsdata->fix.longitude, fix_speed, fix_track, fix_altitude, fix_climb);

        fix_time = gpsdata->fix.time;


        //Move on regular PPRZ GCS map the current  groundstation position
        IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d %f",
                  MSG_DEST,
                  MSG_NAME,
                  MSG_ID, // ac_id
                  0.0, // roll,
                  0.0, // pitch,
                  0.0, // heading
                  gpsdata->fix.latitude,
                  gpsdata->fix.longitude,
                  fix_speed,
                  fix_track, // course
                  fix_altitude,
                  fix_climb,
                  0.0, // agl
                  gpsdata->fix.time,
                  0, // itow
                  0.0); // airspeed


        //Also send this to the aircraft
        new_wp.ac_id = 213;
        new_wp.wp = STAY_WP_INDEX;
        new_wp.lat = gpsdata->fix.latitude;
        new_wp.lon = gpsdata->fix.longitude;
        new_wp.alt = fix_altitude+10;

        if (verbose) g_print ("Now sending MOVE_WAYPOINT to servant AC with ID: %d\n",new_wp.ac_id);
        IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f", new_wp.ac_id, new_wp.wp, new_wp.lat, new_wp.lon, new_wp.alt);
        //A shaddow point for easier debugging, is viewable next to other tht moves
        IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f", new_wp.ac_id, WP_B, new_wp.lat+.00008, new_wp.lon+.00006, 600.);
        //IvyBindMsg(start_track,0,"(NAV_STATUS 1 1 +.*)");
        IvyBindMsg(set_end,0,"(WAYPOINT_MOVED 1 5 +.*)");

        fix_time = gpsdata->fix.time;

    }
    else
    {
        if (verbose)
            g_print ("ignoring gps data: lat %f, lon %f, mode %d, time %f\n", gpsdata->fix.latitude,
                   gpsdata->fix.longitude, gpsdata->fix.mode, gpsdata->fix.time);
    }
}

static gboolean gps_periodic(gpointer data __attribute__ ((unused)))
{
    if (gps_waiting (gpsdata, 500)) {
        if (gps_read (gpsdata) == -1) {
            perror("gps read error");
        } else {
            update_gps(gpsdata, NULL, 0);
        }
    }
    return TRUE;
}

gboolean parse_args(int argc, char** argv)
{
    arg_ac_id = 201;
    verbose = FALSE;
    server = "localhost";
    port = DEFAULT_GPSD_PORT;

#ifdef __APPLE__
    ivy_bus = "224.255.255.255";
#else
    ivy_bus = "127.255.255.255";
#endif

    static const char* usage =
        "Usage: %s [options]\n"
        " Options :\n"
        "   -h --help                                                  Display this help\n"
        "   -v --verbose                                               Display verbose information\n"
        "   --server <gpsd server>                                     e.g. localhost\n"
        "   --port <gpsd port>                                         e.g. 2947\n"
        "   --ivy_bus <ivy bus>                                        e.g. 127.255.255.255\n"
        "   --ac_id <The aircraft ID that should start following you>  e.g. 234\n";

    while (1) {

        static struct option long_options[] = {
            {"ac_id", 1, NULL, 0},
            {"ivy_bus", 1, NULL, 0},
            {"server", 1, NULL, 0},
            {"port", 1, NULL, 0},
            {"help", 0, NULL, 'h'},
            {"verbose", 0, NULL, 'v'},
            {0, 0, 0, 0}
        };
        int option_index = 0;
        int c = getopt_long(argc, argv, "vh",
                            long_options, &option_index);

        if (c == -1)
            break;

        switch (c) {
            case 0:
                switch (option_index) {

                    case 0:
                        arg_ac_id = atol(strdup(optarg)); break;
                    case 1:
                        ivy_bus = strdup(optarg); break;
                    case 2:
                        server = strdup(optarg); break;
                    case 3:
                        port = strdup(optarg); break;
                    default:
                        break;
                }
                break;

            case 'v':
                verbose = TRUE;
                break;

            case 'h':
                fprintf(stderr, usage, argv[0]);
                exit(0);

            default: /* ’?’ */
                g_print ("?? getopt returned character code 0%o ??\n", c);
                fprintf(stderr, usage, argv[0]);
                exit(EXIT_FAILURE);
        }
    }
    return TRUE;
}

/* Called by clicking a button */
void init_follow_me_please( GtkWidget *widget, gpointer data )
{ 
  int TAKEOFF_BLOCK_INDEX = 2;
  int LAND_BLOCK_INDEX =4;

  //Waypoint new_wp;
  new_wp.ac_id = arg_ac_id;

   //If not following, start following
   if (!doFollowMe) {
     doFollowMe = 1;
     gtk_button_set_label(GTK_BUTTON(widget), "Stop Following Me");
     if (verbose) g_print ("I've started following you my master...\n");
     //The takeoff block of the new ardrone_follow flightplan
     IvySendMsg("dl JUMP_TO_BLOCK %d %d", new_wp.ac_id, TAKEOFF_BLOCK_INDEX);
   }
   else {
     doFollowMe = 0;
     gtk_button_set_label(GTK_BUTTON(widget), "Start Following Me");
     if (verbose) g_print ("Your wish is my command, I stopped following you my master...\n");
     //The Land Block
     IvySendMsg("dl JUMP_TO_BLOCK %d %d", new_wp.ac_id, LAND_BLOCK_INDEX);
   }
}

/* Called when aircraft reaches "Block 1" (the second 
 * block defined in the flight plan)
 */
void start_follow(IvyClientPtr app, void *data, int argc, char **argv)
{
// later
}

void destroy( GtkWidget *widget, gpointer data )
{
    gtk_main_quit();
    gtk_widget_destroy(widget);
    exit(0);
}

gint delete_event( GtkWidget *widget,
                   GdkEvent  *event,
                   gpointer   data )
{
  g_print ("Clean exit\n");
  IvyStop();
  exit(0);
  return(FALSE); // false = delete window, FALSE = keep active
}

int main(int argc, char** argv)
{
    int ret = 0;

    if (!parse_args(argc, argv)) {
       return 1; //ERROR_FAIL 
    }
    
    gtk_init(&argc, &argv);

    //Main window
    GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_signal_connect (GTK_OBJECT (window), "delete_event", GTK_SIGNAL_FUNC (delete_event), NULL);
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    gtk_window_set_title (GTK_WINDOW (window), "FMP AppServer");
    gtk_window_set_default_size(GTK_WINDOW(window), 200, 100);
    
    GtkWidget *box1 = gtk_vbox_new(TRUE, 1);
    gtk_container_add (GTK_CONTAINER (window), box1);

    btnStartStop = gtk_button_new_with_label("Plz wait...");
    g_signal_connect(G_OBJECT(btnStartStop), "clicked", G_CALLBACK(init_follow_me_please), 0);
    gtk_box_pack_start(GTK_BOX(box1), btnStartStop, TRUE, TRUE, 0);
    gtk_widget_show (btnStartStop);

    //TODO toggle color between Red/Green see http://ometer.com/gtk-colors.html
    //GdkColor color;
    //gdk_color_parse ("red", &color);
    //gtk_widget_modify_fg (widget, GTK_STATE_NORMAL, &color);
            
    btnQuit = gtk_button_new_with_label("Quit");
    g_signal_connect(G_OBJECT(btnQuit), "clicked", G_CALLBACK(destroy), NULL);

    gtk_box_pack_start(GTK_BOX (box1), btnQuit, TRUE, TRUE, 0);

    /*
    GtkWidget *hbox = gtk_hbox_new(FALSE, 1);
    gtk_container_add (GTK_CONTAINER (box1), hbox);

    lblStatusline = gtk_label_new( "Status:" );
    gtk_box_pack_start(GTK_BOX(hbox), status, FALSE, FALSE, 1);
    gtk_label_set_justify( (GtkLabel*) status, GTK_JUSTIFY_LEFT );

    status = gtk_label_new( status_str );
    gtk_box_pack_start(GTK_BOX(hbox), status, FALSE, FALSE, 1);
    gtk_label_set_justify( (GtkLabel*) status, GTK_JUSTIFY_LEFT );
    */
    gtk_widget_show_all(window);

    GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

    gpsdata = malloc(sizeof(struct gps_data_t));

    g_print ("Connecting to gpsd server %s, port %s\n", server, port);
    g_print ("Using AC ID %d\n", arg_ac_id);

    ret = gps_open(server, port, gpsdata);
    if (ret != 0) {
        perror("error connecting to gpsd");
        return 1;
    }

    gps_stream(gpsdata, WATCH_ENABLE, NULL);

    IvyInit ("GPSd2Ivy", "GPSd2Ivy READY", NULL, NULL, NULL, NULL);
   
    IvyInit("follow_me_please", "follow_me_please READY", NULL, NULL, NULL, NULL);
    IvyBindMsg(textCallback,0,"^follow_me_please hello(.*)");
    IvyBindMsg(start_follow,0,"(NAV_STATUS 1 1 +.*)");
    IvyBindMsg(update_gps,0,"(WAYPOINT_MOVED 1 5 +.*)");
    IvyStart(ivy_bus);

    g_timeout_add(TIMEOUT_PERIOD, gps_periodic, NULL);

    g_main_loop_run(ml);

    (void) gps_stream(gpsdata, WATCH_DISABLE, NULL);
    (void) gps_close (gpsdata);

    free(gpsdata);

    return ret;
    
    /* GTK main */
    gtk_main();

    fprintf(stderr,"Stopped\n");
    return 0;
}
