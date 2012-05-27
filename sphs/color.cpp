#include "color.h"

#include <iostream>

using namespace std;

vec3f rgb(float hue) {
	int hi = hue / 60;
	int f = (hue / 60.0 - (float) hi) * 255.0;
	int q = 255-f;
	// float t = 1.0-(1.0-f) -> t = f
	// S = 255;
	// V = 255;
	// p = 0;
	unsigned int r=0,g=0,b=0;
	switch (hi) {
		case 0:
		case 6:
			r = 255; g = f; b = 0; break;
		case 1:
			r = q; g = 255; b = 0; break;
		case 2:
			r = 0; g = 255; b = f; break;
		case 3:
			r = 0; g = q; b = 255; break;
		case 4:
			r = f; g = 0; b = 255; break;
		case 5:
			r = 255; g = 0; b = q;
	}
// 	cout << "hue = " << hue << endl;
// 	cout << "hue/60.0 = " << hue/60.0 << endl;
// 	cout << "hi = " << hi << endl;
// 	cout << "f = " << f << endl;
// 	cout << "r = " << r << endl;
// 	cout << "g = " << g << endl;
// 	cout << "b = " << b << endl;
	return { r/255.0f, g/255.0f, b/255.0f };
}		


/* vim: set foldmethod=marker: */

