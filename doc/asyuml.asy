// use nice fonts, esp with \tt\bf supported
texpreamble("\usepackage[lighttt]{lmodern}");

// a member variable
struct member {
	string name;
	string visibility;
	void draw(picture to = currentpicture, pair position = (0,0)) {
		label(to, "\tt " + visibility + " " + replace(name, "_", "\_") + "\phantom{\strut}~", position, align=RightSide);
	}
	pair size() {
		picture pic;
		draw(pic);
		return size(pic);
	}
};

// a member function
struct function {
	string name;
	string visibility;
	string after;
	void draw(picture to = currentpicture, pair position = (0,0)) {
		label(to, "\tt " + visibility + " " + replace(name, "_", "\_") + "()\phantom{\strut} " + after + "~", position, align=RightSide);
	}
	pair size() {
		picture pic;
		draw(pic);
		return size(pic);
	}
};

// visibility via bool3
bool3 Public = default;
bool3 Private = false;
bool3 Protected = true;

member make_member(string name, bool3 Access = Protected) {
	member m;
	m.name = name;
	if (Access == Public) m.visibility = "+";
	else if (Access == Private) m.visibility = "-";
	else m.visibility = "\#";
	return m;
}

function make_function(string name, bool3 Access = Public, string suffix = "") {
	function f;
	f.name = name;
	if (Access == Public) f.visibility = "+";
	else if (Access == Private) f.visibility = "-";
	else f.visibility = "\#";
	f.after = suffix;
	return f;
}

// the main part.
// call finalize() when all members/functions are set up to create bounding box information.
struct class {
	string name;
	member[] members;
	function[] functions;
	pair position;
	pair size;
	real[] heights;
	bool track_positions = false;
	class[] bases;
	path[] base_path;

	void add_function(string name, bool3 Access = Public, string suffix = "") {
		functions.push(make_function(name, Access, suffix));
	}
	void add_member(string name, bool3 Access = Protected) {
		members.push(make_member(name, Access));
	}
	int index_of_element(string n) {
		for (int i = 0; i < members.length; ++i)
			if (members[i].name == n)
				return i;
		for (int i = 0; i < functions.length; ++i)
			if (functions[i].name == n)
				return members.length+i;
		if (n == name)
			return heights.length-1;
		return -1;
	}
	pair left_of_element(string name) {
		return position + (0,heights[index_of_element(name)]);
	}
	pair right_of_element(string name) {
		return position + (xpart(size),heights[index_of_element(name)]);
	}
	pair left()   { return position + (0,ypart(size)*.5); }
	pair right()  { return position + (xpart(size),ypart(size)*.5); }
	pair top()    { return position + (xpart(size)*.5,ypart(size)); }
	pair bottom() { return position + (xpart(size)*.5,0); }
	void set_left(pair pos)   { position = pos+(0,-ypart(size)*.5); }
	void set_right(pair pos)  { position = pos+(-xpart(size),-ypart(size)*.5); }
	void set_top(pair pos)    { position = pos-(xpart(size)*.5,ypart(size)); }
	void set_bottom(pair pos) { position = pos+(-xpart(size)*.5,0); }
	void draw_name(picture to = currentpicture, pair pos = (0,0)) {
		label(to, "{\ttfamily\bfseries "+replace(name,"_","\_")+"}\phantom{\strut}", pos);
	}
	pair name_size() {
		picture pic;
		draw_name(pic);
		return size(pic);
	}
	void draw(picture to = currentpicture) {
		pair[] fun_dim = new pair[functions.length];
		pair[] mem_dim = new pair[members.length];
		pair frame_dim = name_size();
		if (track_positions)
			heights = new real[functions.length + members.length + 1];

		// accumulate member variable dimensions
		for (int i = 0; i < members.length; ++i) {
			pair size = members[i].size();
			mem_dim[i] = size;
			frame_dim = (max(xpart(frame_dim), xpart(size)), ypart(frame_dim)+ypart(size));
		}

		// accumulate function dimensions
		for (int i = 0; i < functions.length; ++i) {
			pair size = functions[i].size();
			fun_dim[i] = size;
			frame_dim = (max(xpart(frame_dim), xpart(size)), ypart(frame_dim)+ypart(size));
		}

		// draw frame
		path outer = (0,0)--(xpart(frame_dim),0)--frame_dim--(0,ypart(frame_dim))--cycle;
		size = frame_dim;
		draw(to, shift(position)*outer);

		// draw member variables (from bottom up!)
		pair offset = (0,0);
		if (members.length > 0) {
			offset = position + (0,ypart(mem_dim[members.length-1])*.5);
			for (int i = members.length-1; i >= 0; --i) {
				members[i].draw(to, offset);
				if (track_positions)
					heights[i] = ypart(offset);
				offset = offset + (0,ypart(mem_dim[i]));
			}
			draw(to, offset -(0,ypart(mem_dim[members.length-1])*.5)-- offset + (xpart(frame_dim),-ypart(mem_dim[members.length-1])*.5));
		}
		// draw functions (from bottom up!)
		if (functions.length > 0) {
			if (offset == (0,0))
				offset = position+(0,ypart(fun_dim[functions.length-1])*.5);
			for (int i = functions.length-1; i >= 0; --i) {
				functions[i].draw(to, offset);
				if (track_positions)
					heights[members.length+i] = ypart(offset);
				offset = offset + (0,ypart(fun_dim[i]));
			}
			draw(to, offset -(0,ypart(name_size())*.5)-- offset + (xpart(frame_dim),-ypart(name_size())*.5));
		}
		// and draw name
		draw_name(to, offset + (xpart(frame_dim)*.5,0));
		if (track_positions)
			heights[heights.length-1] = ypart(offset);

		// draw inheritance arrows
		for (int i = 0; i < bases.length; ++i) {
			add(to,arrow(base_path[i],FillDraw(white,black),size=15));
		}
	}
	void add_baseclass(class x, path Path=nullpath, real t = 0.5) {
		bases.push(x);
		if (Path == nullpath) {
			write("nullpath for ", name);
			real ymid = ypart(top()) + t*(ypart(x.bottom()) - ypart(top()));
			base_path.push(top() -- (xpart(top()), ymid) -- (xpart(x.bottom()), ymid) -- x.bottom());
		}
		else {
			write("path for ", name);
			base_path.push(Path);
			}
	}

	void finalize() {
		track_positions = true;
		picture pic;
		draw(pic);
		track_positions = false;
	}
};

class all_classes[];

void finalize_all_classes() {
	for (int i = 0; i < all_classes.length; ++i)
		all_classes[i].finalize();
}

void draw_all_classes() {
	for (int i = 0; i < all_classes.length; ++i)
		all_classes[i].draw();
}

class make_class(string name, pair position = (0,0)) {
	class c;
	c.name = name;
	all_classes.push(c);
	return c;
}

int ConnectionLeft = 0,
	ConnectionRight = 1,
	ConnectionTop = 2,
	ConnectionBottom = 3
	;
path relation_path(pair fro, int fro_side, 
					pair to, int to_side) {
	path p;
	bool fro_left_right = (fro_side == ConnectionLeft || fro_side == ConnectionRight);
	bool to_left_right = (to_side == ConnectionLeft || to_side == ConnectionRight);

	if ((fro_left_right && !to_left_right) ||
		(!fro_left_right && to_left_right)) {
		return fro -- (xpart(to),ypart(fro)) -- to;
	}

	if (fro_side == ConnectionLeft) {
		if (to_side == ConnectionLeft) {
			real min_x = min(xpart(fro), xpart(to));
			p = fro -- (min_x,ypart(fro))-(15,0) -- (min_x,ypart(to))-(15,0) -- to;
		}
		else if (to_side == ConnectionRight) {
			p = fro -- (xpart(fro)+xpart(to-fro)*.5, ypart(fro)) 
					-- (xpart(fro)+xpart(to-fro)*.5, ypart(to))
					-- to;
		}
	}
	else if (fro_side == ConnectionRight) {
		if (to_side == ConnectionLeft) {
			p = fro -- (xpart(fro)+xpart(to-fro)*.5, ypart(fro)) 
					-- (xpart(fro)+xpart(to-fro)*.5, ypart(to))
					-- to;
		}
		else if (to_side == ConnectionRight) {
			real max_x = max(xpart(fro), xpart(to));
			p = fro -- (max_x,ypart(fro))+(15,0) -- (max_x,ypart(to))+(15,0) -- to;
		}
	}
	else if (fro_side == ConnectionTop) {
	}
	else if (fro_side == ConnectionBottom) {
	}
	return p;
}


