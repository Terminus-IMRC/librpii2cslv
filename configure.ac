#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([librpii2cslv], [1.0])
AM_INIT_AUTOMAKE([foreign -W -Wall])
AC_CONFIG_SRCDIR([src/main.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# pkgconfig
AC_ARG_WITH(pkgconfigdir,
            AC_HELP_STRING([--with-pkgconfigdir=DIR],
                           [pkgconfig dir [default=LIBDIR/pkgconfig]]),
                           [pkgconfigdir=${withval}],
                           [pkgconfigdir=${libdir}/pkgconfig])
AC_MSG_NOTICE([pkgconfig directory is ${pkgconfigdir}])
AC_SUBST([pkgconfigdir])

# Checks for programs.
AC_PROG_CC
AM_PROG_AR

# Checks for libraries.

PKG_CHECK_MODULES([BCM_HOST], [bcm_host],
                  [],
                  [librpigrafx_cv_have_bcm_host="no"])
if test "$librpigrafx_cv_have_bcm_host" = "no"; then
  AC_MSG_NOTICE([Adding /opt/vc/lib/pkgconfig to PKG_CONFIG_PATH as bcm_host is not found in current path])
  export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:/opt/vc/lib/pkgconfig"
  PKG_CHECK_MODULES([BCM_HOST], [bcm_host],
                  [librpigrafx_cv_have_bcm_host="yes"],
                  [AC_MSG_ERROR([missing -lbcm_host])])
fi
AC_SUBST([BCM_HOST_CFLAGS])
AC_SUBST([BCM_HOST_LIBS])

AC_CHECK_LIB([wiringPi], [wiringPiSetupGpio],
             [],
             [AC_MSG_ERROR([missing -lwiringPi])])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_MMAP
AC_CHECK_FUNCS([munmap strerror])

LT_INIT
AC_CONFIG_FILES([Makefile include/Makefile src/Makefile test/Makefile
                 librpii2cslv.pc])
AC_OUTPUT
