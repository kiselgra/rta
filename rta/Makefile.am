bin_PROGRAMS = rta

rta_SOURCES = main.cpp cmdline.cpp cmdline.h

rta_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir) -D PKG_LIB_DIR=\"$(pkglibdir)\"
rta_CXXFLAGS = -Wreturn-type
rta_LDADD = -lpthread -L../librta -lrta #$(abs_top_builddir)/librta/.libs/librta.a

#librta_bbvh_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?


