AM_CFLAGS = -pipe -O2 -g -W -Wall -Wextra -I$(top_srcdir)/include/ \
            $(BCM_HOST_CFLAGS)

lib_LTLIBRARIES = librpii2cslv.la

librpii2cslv_la_SOURCES = main.c
librpii2cslv_la_LIBADD = $(BCM_HOST_LIBS)
