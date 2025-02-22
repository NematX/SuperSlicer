///|/ Copyright (c) Prusa Research 2021 - 2022 Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Color_hpp_
#define slic3r_Color_hpp_

#include <array>
#include <algorithm>

#include "Point.hpp"

namespace Slic3r {

class ColorRGB
{
	std::array<float, 3> m_data{1.0f, 1.0f, 1.0f};

public:
	ColorRGB() = default;
	ColorRGB(float r, float g, float b);
	ColorRGB(unsigned char r, unsigned char g, unsigned char b);
	ColorRGB(const ColorRGB& other) = default;

	ColorRGB& operator = (const ColorRGB& other) { m_data = other.m_data; return *this; }

	bool operator == (const ColorRGB& other) const { return m_data == other.m_data; }
	bool operator != (const ColorRGB& other) const { return !operator==(other); }
	bool operator < (const ColorRGB& other) const;
	bool operator > (const ColorRGB& other) const;

	ColorRGB operator + (const ColorRGB& other) const;
	ColorRGB operator * (float value) const;

	const float* const data() const { return m_data.data(); }

	float r() const { return m_data[0]; }
	float g() const { return m_data[1]; }
	float b() const { return m_data[2]; }

	void r(float r) { m_data[0] = std::clamp(r, 0.0f, 1.0f); }
	void g(float g) { m_data[1] = std::clamp(g, 0.0f, 1.0f); }
	void b(float b) { m_data[2] = std::clamp(b, 0.0f, 1.0f); }

	void set(unsigned int comp, float value) {
		assert(0 <= comp && comp <= 2);
		m_data[comp] = std::clamp(value, 0.0f, 1.0f);
	}

	unsigned char r_uchar() const { return static_cast<unsigned char>(m_data[0] * 255.0f); }
	unsigned char g_uchar() const { return static_cast<unsigned char>(m_data[1] * 255.0f); }
	unsigned char b_uchar() const { return static_cast<unsigned char>(m_data[2] * 255.0f); }

	static const ColorRGB BLACK()       { return { 0.0f, 0.0f, 0.0f }; }
	static const ColorRGB BLUE()        { return { 0.0f, 0.0f, 1.0f }; }
	static const ColorRGB BLUEISH()     { return { 0.5f, 0.5f, 1.0f }; }
	static const ColorRGB CYAN()        { return { 0.0f, 1.0f, 1.0f }; }
	static const ColorRGB DARK_GRAY()   { return { 0.25f, 0.25f, 0.25f }; }
	static const ColorRGB DARK_YELLOW() { return { 0.5f, 0.5f, 0.0f }; }
	static const ColorRGB GRAY()        { return { 0.5f, 0.5f, 0.5f }; }
	static const ColorRGB GREEN()       { return { 0.0f, 1.0f, 0.0f }; }
	static const ColorRGB GREENISH()    { return { 0.5f, 1.0f, 0.5f }; }
	static const ColorRGB LIGHT_GRAY()  { return { 0.75f, 0.75f, 0.75f }; }
	static const ColorRGB MAGENTA()     { return { 1.0f, 0.0f, 1.0f }; }
	static const ColorRGB ORANGE()      { return { 0.92f, 0.50f, 0.26f }; }
	static const ColorRGB RED()         { return { 1.0f, 0.0f, 0.0f }; }
	static const ColorRGB REDISH()      { return { 1.0f, 0.5f, 0.5f }; }
	static const ColorRGB YELLOW()      { return { 1.0f, 1.0f, 0.0f }; }
	static const ColorRGB WHITE()       { return { 1.0f, 1.0f, 1.0f }; }

	static const ColorRGB X()           { return { 0.75f, 0.0f, 0.0f }; }
	static const ColorRGB Y()           { return { 0.0f, 0.75f, 0.0f }; }
	static const ColorRGB Z()           { return { 0.0f, 0.0f, 0.75f }; }
};

class ColorRGBA
{
	std::array<float, 4> m_data{ 1.0f, 1.0f, 1.0f, 1.0f };

public:
	ColorRGBA() = default;
	ColorRGBA(float r, float g, float b, float a);
	ColorRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a);
	ColorRGBA(const ColorRGBA& other) = default;

	ColorRGBA& operator = (const ColorRGBA& other) { m_data = other.m_data; return *this; }

	bool operator == (const ColorRGBA& other) const { return m_data == other.m_data; }
	bool operator != (const ColorRGBA& other) const { return !operator==(other); }
	bool operator < (const ColorRGBA& other) const;
	bool operator > (const ColorRGBA& other) const;

	ColorRGBA operator + (const ColorRGBA& other) const;
	ColorRGBA operator * (float value) const;

	const float* const data() const { return m_data.data(); }

	float r() const { return m_data[0]; }
	float g() const { return m_data[1]; }
	float b() const { return m_data[2]; }
	float a() const { return m_data[3]; }

	void r(float r) { m_data[0] = std::clamp(r, 0.0f, 1.0f); }
	void g(float g) { m_data[1] = std::clamp(g, 0.0f, 1.0f); }
	void b(float b) { m_data[2] = std::clamp(b, 0.0f, 1.0f); }
	void a(float a) { m_data[3] = std::clamp(a, 0.0f, 1.0f); }

	void set(unsigned int comp, float value) {
		assert(0 <= comp && comp <= 3);
		m_data[comp] = std::clamp(value, 0.0f, 1.0f);
	}

	unsigned char r_uchar() const { return static_cast<unsigned char>(m_data[0] * 255.0f); }
	unsigned char g_uchar() const { return static_cast<unsigned char>(m_data[1] * 255.0f); }
	unsigned char b_uchar() const { return static_cast<unsigned char>(m_data[2] * 255.0f); }
	unsigned char a_uchar() const { return static_cast<unsigned char>(m_data[3] * 255.0f); }

	bool is_transparent() const { return m_data[3] < 1.0f; }

	static const ColorRGBA BLACK()       { return { 0.0f, 0.0f, 0.0f, 1.0f }; }
	static const ColorRGBA BLUE()        { return { 0.0f, 0.0f, 1.0f, 1.0f }; }
	static const ColorRGBA BLUEISH()     { return { 0.5f, 0.5f, 1.0f, 1.0f }; }
	static const ColorRGBA CYAN()        { return { 0.0f, 1.0f, 1.0f, 1.0f }; }
	static const ColorRGBA DARK_GRAY()   { return { 0.25f, 0.25f, 0.25f, 1.0f }; }
	static const ColorRGBA DARK_YELLOW() { return { 0.5f, 0.5f, 0.0f, 1.0f }; }
	static const ColorRGBA GRAY()		 { return { 0.5f, 0.5f, 0.5f, 1.0f }; }
	static const ColorRGBA GREEN()		 { return { 0.0f, 1.0f, 0.0f, 1.0f }; }
	static const ColorRGBA GREENISH()    { return { 0.5f, 1.0f, 0.5f, 1.0f }; }
	static const ColorRGBA LIGHT_GRAY()  { return { 0.75f, 0.75f, 0.75f, 1.0f }; }
	static const ColorRGBA MAGENTA()     { return { 1.0f, 0.0f, 1.0f, 1.0f }; }
	static const ColorRGBA ORANGE()      { return { 0.923f, 0.504f, 0.264f, 1.0f }; }
	static const ColorRGBA RED()         { return { 1.0f, 0.0f, 0.0f, 1.0f }; }
	static const ColorRGBA REDISH()      { return { 1.0f, 0.5f, 0.5f, 1.0f }; }
	static const ColorRGBA YELLOW()      { return { 1.0f, 1.0f, 0.0f, 1.0f }; }
	static const ColorRGBA WHITE()       { return { 1.0f, 1.0f, 1.0f, 1.0f }; }

	static const ColorRGBA X()           { return { 0.75f, 0.0f, 0.0f, 1.0f }; }
	static const ColorRGBA Y()           { return { 0.0f, 0.75f, 0.0f, 1.0f }; }
	static const ColorRGBA Z()           { return { 0.0f, 0.0f, 0.75f, 1.0f }; }
};

ColorRGB operator * (float value, const ColorRGB& other);
ColorRGBA operator * (float value, const ColorRGBA& other);

ColorRGB lerp(const ColorRGB& a, const ColorRGB& b, float t);
ColorRGBA lerp(const ColorRGBA& a, const ColorRGBA& b, float t);

ColorRGB complementary(const ColorRGB& color);
ColorRGBA complementary(const ColorRGBA& color);

ColorRGB saturate(const ColorRGB& color, float factor);
ColorRGBA saturate(const ColorRGBA& color, float factor);

ColorRGB opposite(const ColorRGB& color);
ColorRGB opposite(const ColorRGB& a, const ColorRGB& b);

bool can_decode_color(const std::string& color);

bool decode_color(const std::string& color_in, ColorRGB& color_out);
bool decode_color(const std::string& color_in, ColorRGBA& color_out);

bool decode_colors(const std::vector<std::string>& colors_in, std::vector<ColorRGB>& colors_out);
bool decode_colors(const std::vector<std::string>& colors_in, std::vector<ColorRGBA>& colors_out);
    
ColorRGBA get_a_color(size_t idx);

std::string encode_color(const ColorRGB& color);
std::string encode_color(const ColorRGBA& color);

ColorRGB  to_rgb(const ColorRGBA& other_rgba);
ColorRGBA to_rgba(const ColorRGB& other_rgb);
ColorRGBA to_rgba(const ColorRGB& other_rgb, float alpha);

typedef struct {
	double h;       // angle in degrees
	double s;       // a fraction between 0 and 1
	double v;       // a fraction between 0 and 1
} hsv;

// Color mapping of a value into RGB false colors.
inline Vec3f value_to_rgbf(float minimum, float maximum, float value) 
{
    float ratio = 2.0f * (value - minimum) / (maximum - minimum);
    float b = std::max(0.0f, (1.0f - ratio));
    float r = std::max(0.0f, (ratio - 1.0f));
    float g = 1.0f - b - r;
    return Vec3f { r, g, b };
}

//utility color methods
hsv         rgb2hsv(const ColorRGB &in);
ColorRGB    hsv2rgb(const hsv &in);
uint32_t    hex2int(const std::string &hex);
std::string int2hex(uint32_t int_color);
ColorRGB    int2rgb(uint32_t int_color);
uint32_t    rgb2int(const ColorRGB &rgb_color);
uint32_t	change_endian_int24(uint32_t int_color);

// Color mapping of a value into RGB false colors.
inline Vec3i32 value_to_rgbi(float minimum, float maximum, float value)
{
    return (value_to_rgbf(minimum, maximum, value) * 255).cast<int>();
}

struct ColorReplace
{
    std::string color_to_replace_str;
    ColorRGB   color_to_replace;
    std::string   new_color_str;
    ColorRGB   new_color;
	// true if colorRGB exists, false if a string isn't a hash-color #00FF00
	bool is_valid = false;
};
struct ColorReplaces{
	std::vector<ColorReplace> changes;
	void add(const std::string&, const std::string&);
	void add(const ColorRGB&, const ColorRGB&);
	void add(const uint32_t&, const uint32_t&);
    void add(const std::string &sold, const uint32_t &inew);
	std::optional<ColorReplace> has_key(const ColorRGB&) const;
	std::optional<ColorReplace> has_value(const ColorRGB&) const;
	std::optional<ColorReplace> has_key(const std::string&) const;
	std::optional<ColorReplace> has_value(const std::string&) const;
};

ColorRGBA picking_decode(unsigned int id);
unsigned int picking_encode(unsigned char r, unsigned char g, unsigned char b);
// Produce an alpha channel checksum for the red green blue components. The alpha channel may then be used to verify, whether the rgb components
// were not interpolated by alpha blending or multi sampling.
unsigned char picking_checksum_alpha_channel(unsigned char red, unsigned char green, unsigned char blue);

} // namespace Slic3r

#endif /* slic3r_Color_hpp_ */
