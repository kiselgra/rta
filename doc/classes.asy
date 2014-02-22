import asyuml;

class accel_struct = make_class("acceleration_structure");
accel_struct.functions.push(make_function("triangle_ptr", Public));
accel_struct.functions.push(make_function("triangle_count", Public));

class ctor = make_class("acceleration_structure_constructor");
ctor.functions.push(make_function("build", Public, "= 0"));

class ftl = make_class("flat_triangle_list");
ftl.members.push(make_member("triangle*", Public));
ftl.members.push(make_member("triangles", Public));

class rt = make_class("raytracer");
rt.functions.push(make_function("setup_rays", Public, "= 0"));
rt.functions.push(make_function("prepare_bvh_for_trav", Public, "= 0"));
rt.functions.push(make_function("trace", Public, "= 0"));
rt.functions.push(make_function("copy", Public, "= 0"));

class brt = make_class("basic_raytracer");
brt.members.push(make_member("raygen*", Protected));
brt.members.push(make_member("bouncer*", Protected));
brt.members.push(make_member("cpu_bouncer*", Protected));
brt.members.push(make_member("accel_struct*", Protected));
brt.members.push(make_member("timings", Protected));
brt.functions.push(make_function("trace", Public));

class bouncer = make_class("bouncer");
bouncer.functions.push(make_function("bounce", Public, "= 0"));
bouncer.functions.push(make_function("trace_further_bounces", Public, "= 0"));
bouncer.functions.push(make_function("new_pass", Public));

class cpu_bouncer = make_class("cpu_ray_bouncer");
cpu_bouncer.functions.push(make_function("save_intersection", Public));
cpu_bouncer.members.push(make_member("last_intersection", Protected));

class pic = make_class("primary_intersection_collector");
pic.functions.push(make_function("trace_further_bounces", Public));
pic.functions.push(make_function("bounce", Public));
pic.functions.push(make_function("intersection", Public));

class lc = make_class("lighting_collector");
lc.functions.push(make_function("reset", Public));
lc.functions.push(make_function("shade", Public));
lc.functions.push(make_function("add_pointlight", Public));
lc.functions.push(make_function("save", Public));
lc.functions.push(make_function("triangle_ptr", Public));
lc.members.push(make_member("lights", Public));
lc.members.push(make_member("res", Protected));
lc.members.push(make_member("last_intersection*", Protected));
lc.members.push(make_member("triangles*", Protected));

class ddi = make_class("direct_diffuse_illumination");
ddi.functions.push(make_function("bounce", Public));
ddi.functions.push(make_function("trace_further_bounces", Public));


finalize_all_classes();
/////

ctor.position = (xpart(accel_struct.right())-xpart(ctor.size),
                 -ypart(ctor.size)-40);

ftl.position = (xpart(accel_struct.right())-xpart(ftl.size),
                ypart(ctor.bottom())-ypart(ftl.size)-40);


rt.position = (xpart(accel_struct.size)+50);

brt.position = (xpart(rt.position),
                -ypart(brt.size)-40);

bouncer.position = (xpart(rt.position)+xpart(rt.size)+50,
                    ypart(rt.position));

cpu_bouncer.position = (xpart(bouncer.position),
                        -ypart(cpu_bouncer.size)-40);

pic.position = (xpart(cpu_bouncer.position),
                ypart(cpu_bouncer.position)-ypart(pic.size)-40);

lc.position = (xpart(pic.position)+xpart(pic.size)+50,
               ypart(pic.position));

ddi.position = (xpart(pic.position),
                ypart(pic.position)-ypart(ddi.size)-40);

brt.add_baseclass(rt);
cpu_bouncer.add_baseclass(bouncer);
pic.add_baseclass(cpu_bouncer);
ddi.add_baseclass(pic);
ddi.add_baseclass(lc);

/////

//accel_struct.draw();
//ctor.draw();
//ftl.draw();
//rt.draw();
//brt.draw();
//bouncer.draw();
draw_all_classes();
