/**
 *	@class 		RGBColor
 *	@author 	Christian Pfitzner
 *	@date 		2012-10-23
 *	@version 	1.0
 *
 *	@todo 		unreviewed
 *
 *	Class for rgb color
 */

#ifndef __RGBCOLOR__
#define __RGBCOLOR__

#include <ostream>

/**
 * @namespace 	obviously
 */
namespace obvious
{

class RGBColor
{
	public:
		/*
		 * Constructor
		 */
		/**
		 * Function to set up object of RGBColor
		 * @param 		r		red color value 	(default = 0)
		 * @param 		g		green color value 	(default = 0)
		 * @param 		b		blue color value 	(default = 0)
		 */
		RGBColor(const unsigned char r = 0, const unsigned char g = 0, const unsigned char b = 0)
			: 	m_r(r),	m_g(g),	m_b(b) { }

                //! copy constructor
                RGBColor(const RGBColor& rgb) : m_r(rgb.m_r), m_g(rgb.m_g), m_b(rgb.m_b) { }

		/*---------------------------------------------------------------------
		 * Functions to set value
		 */
		/**
		 * Function to set red color value
		 * @param 		r		red color value
		 */
		void setR(const unsigned char r) { m_r = r; }
		/**
		 * Function to set green color value
		 * @param 		g		green
		 */
		void setG(const unsigned char g) { m_g = g; }
		/**
		 * Function to set blue color value
		 * @param 		b		blue color value
		 */
		void setB(const unsigned char b) { m_b = b; }

		/*---------------------------------------------------------------------
		 * Function to get value
		 */
		/**
		 * Function to return red color value
		 * @return		red color value
		 */
		unsigned char r(void) const {return m_r; }
		/**
		 * Function to return green color value
		 * @return		green color value
		 */
		unsigned char g(void) const {return m_g; }
		/**
		 * Function to return blue color value
		 * @return		blue color value
		 */
		unsigned char b(void) const {return m_b; }

		/*---------------------------------------------------------------------
		 * Function OVERLOAD
		 */
		/**
		 * Overloaded function for output stream of RGBColor object
		 * @param 		ostr	output stream
		 * @param 		rgb 	object of class RGBColor
		 * @return		output stream
		 */
		friend std::ostream &operator<<(std::ostream &ostr, RGBColor &rgb)
		{
			ostr << "RGB: " << rgb.m_r << ", " << rgb.m_g << ", " << rgb.m_b << std::endl;
			return ostr;
		}

	private:
		unsigned char m_r;					/**< color value for red */
		unsigned char m_g;					/**< color value for green */
		unsigned char m_b;					/**< color value for blue */
};

};

#endif /* __RGBCOLOR_ */
