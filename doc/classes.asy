import asyuml;

class accel_struct = make_class("acceleration_structure");
accel_struct.add_function("triangle_ptr", Public);
accel_struct.add_function("triangle_count", Public);

class ctor = make_class("acceleration_structure_constructor");
ctor.add_function("build", Public, "= 0");

class ftl = make_class("flat_triangle_list");
ftl.add_member("triangle*", Public);
ftl.add_member("triangles", Public);

class rt = make_class("raytracer");
rt.add_function("setup_rays", Public, "= 0");
rt.add_function("prepare_bvh_for_trav", Public, "= 0");
rt.add_function("trace", Public, "= 0");
rt.add_function("copy", Public, "= 0");

class brt = make_class("basic_raytracer");
brt.add_member("accel_struct*", Protected);
brt.add_member("raygen*", Protected);
brt.add_member("bouncer*", Protected);
brt.add_member("cpu_bouncer*", Protected);
brt.add_member("timings", Protected);
brt.add_function("trace", Public);
brt.add_function("ray_generator");
brt.add_function("ray_bouncer");
brt.add_function("force_cpu_bouncer");
brt.add_function("acceleration_structure");

class bouncer = make_class("bouncer");
bouncer.add_function("bounce", Public, "= 0");
bouncer.add_function("trace_further_bounces", Public, "= 0");
bouncer.add_function("new_pass", Public);

class cpu_bouncer = make_class("cpu_ray_bouncer");
cpu_bouncer.add_function("save_intersection", Public);
cpu_bouncer.add_member("last_intersection", Protected);

class pic = make_class("primary_intersection_collector");
pic.add_function("trace_further_bounces", Public);
pic.add_function("bounce", Public);
pic.add_function("intersection", Public);

class lc = make_class("lighting_collector");
lc.add_function("reset", Public);
lc.add_function("shade", Public);
lc.add_function("add_pointlight", Public);
lc.add_function("save", Public);
lc.add_function("triangle_ptr", Public);
lc.add_member("lights", Public);
lc.add_member("res", Protected);
lc.add_member("last_intersection*", Protected);
lc.add_member("triangles*", Protected);

class ddi = make_class("direct_diffuse_illumination");
ddi.add_function("bounce", Public);
ddi.add_function("trace_further_bounces", Public);

class rgen = make_class("ray_generator");
rgen.add_member("raydata", Public);
rgen.add_function("generate_rays", Public, "= 0");
rgen.add_function("res_x", Public);
rgen.add_function("res_y", Public);
rgen.add_function("origin", Public);
rgen.add_function("origin", Public);
rgen.add_function("direction", Public);
rgen.add_function("direction", Public);

class rgen_sh = make_class("cam_ray_generator_shirley");
rgen_sh.add_member("aspect", Protected);
rgen_sh.add_member("fovy", Protected);
rgen_sh.add_member("position", Protected);
rgen_sh.add_member("dir", Protected);
rgen_sh.add_member("up", Protected);
rgen_sh.add_function("generate_ray_dir", Protected);
rgen_sh.add_function("generate_ray_dir_from_cam_parameters", Protected);
rgen_sh.add_function("setup", Public);

finalize_all_classes();

/////

ctor.position = (xpart(accel_struct.right())-xpart(ctor.size)-60,
                 -ypart(ctor.size)-40);

ftl.set_right(ctor.left());
ftl.position = (xpart(ftl.position), ypart(accel_struct.top())-ypart(ftl.size));


rt.position = (xpart(accel_struct.size)+50);
brt.set_top(rt.bottom() + (0,-30));

bouncer.position = (xpart(rt.position)+xpart(rt.size)+50,
                    ypart(rt.position));
cpu_bouncer.set_top(bouncer.bottom()+(0,-50));
pic.set_top(cpu_bouncer.bottom()+(0,-50));

lc.position = (xpart(pic.position)+xpart(pic.size)+50,
               ypart(pic.position));
ddi.set_top(pic.bottom()+(0,-50));

rgen.position=(xpart(accel_struct.right())-xpart(rgen.size)-50,
               ypart(ctor.bottom())-ypart(rgen.size)-100);

rgen_sh.set_top(rgen.bottom() + (0,-30));


/////

brt.add_baseclass(rt);
cpu_bouncer.add_baseclass(bouncer);
pic.add_baseclass(cpu_bouncer);
ddi.add_baseclass(pic);
ddi.add_baseclass(lc);
rgen_sh.add_baseclass(rgen);

/////

draw_all_classes();

/////

draw(relation_path(ctor.left_of_element("build"), ConnectionLeft,
					ftl.bottom(), ConnectionBottom),
		Arrow);
draw(relation_path(ctor.right_of_element("build"), ConnectionRight,
					accel_struct.right(), ConnectionRight),
		Arrow);

draw(relation_path(lc.left_of_element("last_intersection*"), ConnectionLeft,
					cpu_bouncer.right_of_element("last_intersection"), ConnectionRight),
		Arrow);

draw(relation_path(brt.left_of_element("accel_struct*"), ConnectionLeft,
					accel_struct.right_of_element("acceleration_structure"), ConnectionRight),
		Arrow);
draw(relation_path(brt.left_of_element("raygen*"), ConnectionLeft,
					rgen.top(), ConnectionTop),
		Arrow);
draw(relation_path(brt.right_of_element("bouncer*"), ConnectionRight,
					bouncer.left_of_element("bouncer"), ConnectionLeft),
		Arrow);
draw(relation_path(brt.right_of_element("cpu_bouncer*"), ConnectionRight,
					cpu_bouncer.left_of_element("cpu_ray_bouncer"), ConnectionLeft),
		Arrow);

draw(lc.right_of_element("triangles*") -- lc.right_of_element("triangles*")+(15,0)
										-- (xpart(lc.right_of_element("triangles*"))+15, ypart(rt.top())+15)
										-- (xpart(accel_struct.left())-15, ypart(rt.top())+15)
										-- accel_struct.left_of_element("triangle_ptr")-(15,0)
										-- accel_struct.left_of_element("triangle_ptr"),
		Arrow);

/////
/////
/////

real min_x = xpart(ftl.left())-15;
real max_x = xpart(lc.right())+30;
real min_y = ypart(rgen_sh.bottom())-30;
real max_y = ypart(rt.top())+30;
draw((min_x,max_y) -- (max_x,max_y));
label("\Large librta", (min_x,max_y), align=Relative(S+E));
draw((min_x,min_y) -- (max_x,min_y));
label("\Large bbvh", (min_x,min_y), align=Relative(S+E));

/////
/////
/////

all_classes = new class[];

class binary_bvh_node = make_class("binary_bvh::node");
binary_bvh_node.add_member("type_left_elems", Public);
binary_bvh_node.add_member("right_tris", Public);
binary_bvh_node.add_member("box", Public);
binary_bvh_node.add_function("inner", Public);
binary_bvh_node.add_function("make_inner", Public);
binary_bvh_node.add_function("make_leaf", Public);
binary_bvh_node.add_function("left", Public);
binary_bvh_node.add_function("elems", Public);
binary_bvh_node.add_function("right", Public);
binary_bvh_node.add_function("tris", Public);
binary_bvh_node.add_function("split_axis", Public);

class binary_bvh = make_class("binary_bvh");
binary_bvh.add_member("nodes", Public);
binary_bvh.add_member("triangles", Public);
binary_bvh.add_member("take_node_array", Public);
binary_bvh.add_member("take_triangle_array", Public);
binary_bvh.add_member("trinangle_ptr", Public);
binary_bvh.add_member("trinangle_count", Public);

class binary_bvh_sa_node = make_class("binary_bvh_with_split_axis::node");
binary_bvh_sa_node.add_function("split_axis", Public);
binary_bvh_sa_node.add_member("axis", Public);

class binary_bvh_sa = make_class("binary_bvh_with_split_axis");
binary_bvh_sa.add_member("nodes", Public);
binary_bvh_sa.add_member("triangles", Public);
binary_bvh_sa.add_function("take_node_array", Public);
binary_bvh_sa.add_function("take_triangle_array", Public);
binary_bvh_sa.add_function("trinangle_ptr", Public);
binary_bvh_sa.add_function("trinangle_count", Public);

class bbvh_no_bias = make_class("bbvh_no_bias");
bbvh_no_bias.add_function("apply");

class bbvh_ctor_median = make_class("bbvh_constructor_using_median");
bbvh_ctor_median.add_member("nodes");
bbvh_ctor_median.add_member("max_tris_per_node");
bbvh_ctor_median.add_member("triangles");
bbvh_ctor_median.add_member("median_used");
bbvh_ctor_median.add_member("median", Public);
bbvh_ctor_median.add_function("build");
bbvh_ctor_median.add_function("tri_sort", Protected);
bbvh_ctor_median.add_function("tri_sort_[xyz]", Protected);
bbvh_ctor_median.add_function("build_om", Protected);
bbvh_ctor_median.add_function("build_sm", Protected);
bbvh_ctor_median.add_function("compute_spatial_median", Protected);

class bbvh_tracer = make_class("bbvh_tracer");
bbvh_tracer.add_function("acceleration_structure");
bbvh_tracer.add_member("bvh");

class bbvh_dis_tracer = make_class("bbvh_direct_is_tracer");
bbvh_dis_tracer.add_function("trace_rays");
bbvh_dis_tracer.add_function("trace_ray");
bbvh_dis_tracer.add_function("copy");

class bbvh_cis_tracer = make_class("bbvh_child_is_tracer");
bbvh_cis_tracer.add_function("trace_rays");
bbvh_cis_tracer.add_function("trace_ray");
bbvh_cis_tracer.add_function("copy");


finalize_all_classes();

binary_bvh.set_top(accel_struct.bottom() + (110,min_y-60));
binary_bvh_node.set_top(binary_bvh.left() + (-xpart(binary_bvh.size)+60,20));

binary_bvh_sa.set_top(binary_bvh.bottom() + (xpart(binary_bvh.size)*.5+20,-40));
binary_bvh_sa_node.set_top(binary_bvh_sa.left() + (-40,-80));

bbvh_ctor_median.set_top((xpart(ftl.bottom()),ypart(binary_bvh.top())));
bbvh_ctor_median.position = (xpart(ftl.left()), ypart(bbvh_ctor_median.position));

bbvh_no_bias.set_top(bbvh_ctor_median.bottom()+(xpart(bbvh_ctor_median.size)*.5-xpart(bbvh_no_bias.size)*.5,-20));

bbvh_tracer.set_top((xpart(binary_bvh_sa.right()+.5*binary_bvh_sa.size+10), ypart(binary_bvh.top())+40));
bbvh_dis_tracer.set_top((xpart(bbvh_tracer.left())+10, ypart(bbvh_tracer.bottom())-60));
bbvh_cis_tracer.set_top((xpart(bbvh_tracer.right())-10, ypart(bbvh_tracer.bottom())-60));

/////

binary_bvh.add_baseclass(accel_struct, t=0.95);
binary_bvh_sa.add_baseclass(binary_bvh_node, 
								Path=	binary_bvh_sa.top()
										--(xpart(binary_bvh_sa.top()),ypart(binary_bvh.top())+20)
										--binary_bvh.top() + (0,20)
										--binary_bvh.top()
										--binary_bvh.base_path[0]);
binary_bvh_sa_node.add_baseclass(binary_bvh_node);
bbvh_ctor_median.add_baseclass(ctor, 0.95);
bbvh_tracer.add_baseclass(brt);
bbvh_dis_tracer.add_baseclass(bbvh_tracer);
bbvh_cis_tracer.add_baseclass(bbvh_tracer);

/////

draw(relation_path(binary_bvh.left_of_element("nodes"), ConnectionLeft,
					binary_bvh_node.top(), ConnectionTop),
		Arrow);
draw(relation_path(binary_bvh_sa.left_of_element("nodes"), ConnectionLeft,
					binary_bvh_sa_node.top(), ConnectionTop),
		Arrow);
draw(relation_path(bbvh_ctor_median.right_of_element("build_om"), ConnectionRight,
					bbvh_no_bias.right_of_element("apply"), ConnectionRight),
		Arrow);
draw(relation_path(bbvh_ctor_median.right_of_element("build_sm"), ConnectionRight,
					bbvh_no_bias.right_of_element("apply"), ConnectionRight),
		Arrow);

draw_all_classes();


/////
/////
/////

max_y = min_y;
min_y = ypart(binary_bvh_sa_node.bottom())-30;
draw((min_x,min_y) -- (max_x,min_y));
label("\Large sbvh", (min_x,min_y), align=Relative(S+E));

/////
/////
/////

all_classes = new class[];

class stackless_bvh = make_class("stackless_bvh");
stackless_bvh.add_member("nodes", Public);
stackless_bvh.add_member("triangles", Public);
stackless_bvh.add_function("take_triangle_array", Public);
stackless_bvh.add_function("trinangle_ptr", Public);
stackless_bvh.add_function("trinangle_count", Public);

class stackless_bvh_node = make_class("stackless_bvh::node");
stackless_bvh_node.add_member("type_parent", Public);
stackless_bvh_node.add_member("children_tris", Public);
stackless_bvh_node.add_member("box", Public);
stackless_bvh_node.add_function("inner", Public);
stackless_bvh_node.add_function("make_inner", Public);
stackless_bvh_node.add_function("make_leaf", Public);
stackless_bvh_node.add_function("parent", Public);
stackless_bvh_node.add_function("children", Public);
stackless_bvh_node.add_function("tris", Public);
stackless_bvh_node.add_function("elems", Public);
stackless_bvh_node.add_function("split_axis", Public);
stackless_bvh_node.add_function("incorporate", Public);

class sbvh_constructor = make_class("sbvh_constructor");
sbvh_constructor.add_function("build");

class preorder_stackless_bvh = make_class("preorder_stackless_bvh");
preorder_stackless_bvh.add_member("nodes", Public);
preorder_stackless_bvh.add_member("triangles", Public);
preorder_stackless_bvh.add_function("take_triangle_array", Public);
preorder_stackless_bvh.add_function("trinangle_ptr", Public);
preorder_stackless_bvh.add_function("trinangle_count", Public);

class preorder_stackless_bvh_node = make_class("preorder_stackless_bvh::node");
preorder_stackless_bvh_node.add_member("type_elems");
preorder_stackless_bvh_node.add_member("skip_tris");
preorder_stackless_bvh_node.add_member("box");
preorder_stackless_bvh_node.add_function("inner");
preorder_stackless_bvh_node.add_function("make_inner");
preorder_stackless_bvh_node.add_function("make_leaf");
preorder_stackless_bvh_node.add_function("elems");
preorder_stackless_bvh_node.add_function("skip");
preorder_stackless_bvh_node.add_function("tris");

finalize_all_classes();

/////

stackless_bvh.set_top((0, min_y-100));
stackless_bvh_node.set_left(stackless_bvh.right());
sbvh_constructor.set_left(stackless_bvh_node.right());
preorder_stackless_bvh.set_left(sbvh_constructor.right());
preorder_stackless_bvh_node.set_left(preorder_stackless_bvh.right());



/////

/////

draw_all_classes();
