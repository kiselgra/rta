#ifndef __RTA_IMAGE_H__ 
#define __RTA_IMAGE_H__ 

#include <png.h>

#include <stdexcept>
#include <string>

#include <png++/png.hpp>
#include <tr1/memory>

namespace rta
{

	template<typename T, unsigned int C> class image
	{
	public:
		image(const unsigned int w, const unsigned int h);
		explicit image(const image &o);
		~image();

		inline void save_png(const std::string &filename);

		unsigned int w,h;		
		T *data;					//!< data==0 --> invalid

		inline T& pixel(const unsigned int x, const unsigned int y)                                   { return data[y * w * C + x * C]; }
		inline T& pixel(const unsigned int x, const unsigned int y, const unsigned int c)             { return data[y * w * C + x * C + c]; }
		inline const T& pixel(const unsigned int x, const unsigned int y) const                       { return data[y * w * C + x * C]; }
		inline const T& pixel(const unsigned int x, const unsigned int y, const unsigned int c) const { return data[y * w * C + x * C + c]; }
		typedef std::tr1::shared_ptr<image<T, C> > ref;
	};

		
	//! convert channel numbers to png++ pixel types
	template<unsigned int C> class channels_to_pixel {};
	template<> class channels_to_pixel<3> {	public: typedef png::rgb_pixel pixel_t; };
	template<> class channels_to_pixel<4> {	public: typedef png::rgba_pixel pixel_t; };

	//! create a png++ pixel depending on the number of channels, and fill it with data
	template<typename T, unsigned int C> class fill_channel_pixel {};
	template<typename T> class fill_channel_pixel<T,3> {	public: static inline channels_to_pixel<3>::pixel_t fill(T *pos) {  return channels_to_pixel<3>::pixel_t(pos[0], pos[1], pos[2]); }  };
	template<typename T> class fill_channel_pixel<T,4> {	public: static inline channels_to_pixel<4>::pixel_t fill(T *pos) {  return channels_to_pixel<4>::pixel_t(pos[0], pos[1], pos[2], pos[3]); }  };



	template<typename T, unsigned int C> image<T, C>::image(const unsigned int w, const unsigned int h)
	: w(w), h(h), data(0)
	{
		if (w > 0 && h > 0)
			data = new T[w * h * C];
	}

	template<typename T, unsigned int C> inline void image<T, C>::save_png(const std::string &filename)
	{
		png::image<typename channels_to_pixel<C>::pixel_t> image(w,h);
		unsigned int pos = 0;
		for (unsigned int y = 0; y < h; ++y)
			for (unsigned int x = 0; x < w; ++x)
			{
				image.set_pixel(x, y, fill_channel_pixel<T, C>::fill(data+pos));
				pos += C;
			}
		image.write(filename);
	}

	template<typename T, unsigned int C> image<T, C>::~image()
	{
		delete [] data;
		w = h = 0;
	}

	//! catch stupid errors
	template<typename T> class image<T,0> { };

}



#endif

