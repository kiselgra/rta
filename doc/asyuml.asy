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

member make_member(string name, bool3 Access) {
	member m;
	m.name = name;
	if (Access == Public) m.visibility = "+";
	else if (Access == Private) m.visibility = "-";
	else m.visibility = "\#";
	return m;
}

function make_function(string name, bool3 Access, string suffix = "") {
	function f;
	f.name = name;
	if (Access == Public) f.visibility = "+";
	else if (Access == Private) f.visibility = "-";
	else f.visibility = "#";
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
	int index_of_element(string name) {
		for (int i = 0; i < members.length; ++i)
			if (members[i].name == name)
				return i;
		for (int i = 0; i < functions.length; ++i)
			if (functions[i].name == name)
				return members.length+i;
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
			heights = new real[functions.length + members.length];

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
				heights[members.length+i] = ypart(offset);
				offset = offset + (0,ypart(fun_dim[i]));
			}
			draw(to, offset -(0,ypart(name_size())*.5)-- offset + (xpart(frame_dim),-ypart(name_size())*.5));
		}
		// and draw name
		draw_name(to, offset + (xpart(frame_dim)*.5,0));
	}
	void finalize() {
		track_positions = true;
		picture pic;
		draw(pic);
		track_positions = false;
	}
};

class make_class(string name, pair position = (0,0)) {
	class c;
	c.name = name;
	return c;
}


