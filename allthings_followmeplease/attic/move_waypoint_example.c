
#include <stdio.h>
#include <stdlib.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <gtk-2.0/gtk/gtk.h>
/*#include <gtk/gtk.h> is what the previous line should read when compiling
with pkg-config --cflags --libs gtk+-2.0 */

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

//#include "nextwp.c"
//#include "waypoint.h"

/* Called by clicking a button */
void init_follow_me_please( GtkWidget *widget, gpointer data )
{
    fprintf(stderr, "Getting ready to follow you master...\n");
    /* Call function get_next_waypoint (defined in nexcwp.c)
     * which returns a waypoint. */
    Waypoint new_wp = get_next_waypoint();
    IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f", \
            new_wp.ac_id, 4, new_wp.lat, new_wp.lon, new_wp.alt);
}

static void on_button_clicked (GtkWidget *widget, gpointer data) {
  g_message("Stop bugging me, please");
  //int foo = 42;
  //IvySendMsg("ME HELLO_WORLD 1234 5678 %d", foo);
}

int main ( int argc, char** argv) {

  gtk_init(&argc, &argv);

  IvyInit ("FollowMePlease", "FollowMePlease READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "Follow Me Please");

  GtkWidget* button = gtk_toggle_button_new_with_label( "Follow Me Please" );
  gtk_container_add (GTK_CONTAINER (window), button);
  g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (on_button_clicked), NULL);

  gtk_widget_show_all(window);

  gtk_main();
  return 0;
}

