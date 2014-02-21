import asyuml;

class accel_struct = make_class("acceleration_structure");
accel_struct.functions.push(make_function("triangle_ptr", Public));
accel_struct.functions.push(make_function("triangle_count", Public));
accel_struct.finalize();

class ctor = make_class("acceleration_structure_constructor");
ctor.functions.push(make_function("build", Public, "= 0"));
ctor.finalize();

class ftl = make_class("flat_triangle_list");
ftl.members.push(make_member("triangle*", Public));
ftl.members.push(make_member("triangles", Public));
ftl.finalize();

class rt = make_class("raytracer");
rt.functions.push(make_function("setup_rays", Public, "= 0"));
rt.functions.push(make_function("prepare_bvh_for_trav", Public, "= 0"));
rt.functions.push(make_function("trace", Public, "= 0"));
rt.functions.push(make_function("copy", Public, "= 0"));
rt.finalize();

class brt = make_class("basic_raytracer");
brt.members.push(make_member("raygen*", Protected));
brt.members.push(make_member("bouncer*", Protected));
brt.members.push(make_member("cpu_bouncer*", Protected));
brt.members.push(make_member("accel_struct*", Protected));
brt.members.push(make_member("timings", Protected));
brt.functions.push(make_function("trace", Public));
brt.finalize();

/////

ctor.position = (xpart(accel_struct.right())-xpart(ctor.size),
                 -ypart(ctor.size)-20);

ftl.position = (xpart(accel_struct.right())-xpart(ftl.size),
                ypart(ctor.bottom())-ypart(ftl.size)-20);


rt.position = (xpart(accel_struct.size)+50);

brt.position = (xpart(rt.position),
                -ypart(brt.size)-20);

accel_struct.draw();
ctor.draw();
ftl.draw();
rt.draw();
brt.draw();

