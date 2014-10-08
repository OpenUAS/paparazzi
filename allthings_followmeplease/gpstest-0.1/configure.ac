AC_INIT(gpstest, 0.1, jmoore@zedstar.org)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([src/])
AM_CONFIG_HEADER([src/config.h])

AC_DEFINE([PROGNAME],["gpstest"],["gps test program"])
AC_DEFINE([READ_FREQ],[1],["seconds to wait between reads"])

# Checks for programs.
AC_PROG_CC

# Checks for header files.
AC_HEADER_STDC

# Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_MALLOC

dnl ========= check for GPSD ===================================================
AC_ARG_WITH(gpsd,
	[ --with-gpsd=<path> prefix of gpsd installation.],
	[ GPSD_PREFIX=$with_gpsd ],
	[ GPSD_PREFIX=/usr ])
	
CPPFLAGS="$CPPFLAGS -I${GPSD_PREFIX}/include"

# test for gps header and library
AC_CHECK_HEADERS(gps.h, , AC_MSG_ERROR(*** gps.h not found))
AC_CHECK_LIB(gps, gps_open, , AC_MSG_ERROR(*** libgps not found))
	
AC_SUBST(GPSD_PREFIX)
GPSD_LIBS="-L${GPSD_PREFIX}/lib -lgps"
GPSD_CFLAGS="-I${GPSD_PREFIX}/include"

AC_SUBST(GPSD_LIBS)
AC_SUBST(GPSD_CFLAGS)

# check for glib
PKG_CHECK_MODULES(GLIB, [glib-2.0])

AC_CONFIG_FILES([Makefile src/Makefile logs/Makefile])

AC_OUTPUT
