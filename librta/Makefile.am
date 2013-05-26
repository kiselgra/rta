lib_LTLIBRARIES = librta.la

librta_la_SOURCES = aabb.h  \
					basic_types.h  \
					image.h  \
					intersect.h  \
					plugins.h plugins.cpp\
					raytrav.h  \
					template-magic.h  \
					tri.h \
					ocl.h ocl.cpp \
					rta-config.h \
					librta.h \
					wall-timer.h

library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)/librta
library_include_HEADERS = aabb.h  basic_types.h  image.h  intersect.h  plugins.h  raytrav.h  template-magic.h  tri.h ocl.h rta-config.h librta.h wall-timer.h

# setting c preprocessor flags for main project
librta_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -D PKG_LIB_DIR=\"$(pkglibdir)\"
librta_la_CXXFLAGS = -Wreturn-type

