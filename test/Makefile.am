AM_CFLAGS = -pipe -O2 -g -W -Wall -Wextra -I$(top_srcdir)/include/ \
            $(BCM_HOST_CFLAGS)

check_PROGRAMS = test_simple

nodist_test_simple_SOURCES = test_simple.c
test_simple_LDADD = $(top_builddir)/src/.libs/librpii2cslv.a $(BCM_HOST_LIBS) \
                    -lwiringPi -lpthread
