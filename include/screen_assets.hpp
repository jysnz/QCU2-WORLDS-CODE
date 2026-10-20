#pragma once
#include <cstdint>

// Brain-screen assets, generated into src/screen_assets.cpp by tools/make_screen_assets.py.

// Palettized image: one byte per pixel, indexing a 0xAARRGGBB palette (alpha 0 = transparent).
struct Image {
    int w, h;
    const std::uint32_t *palette;
    const std::uint8_t *pixels;
};

// QCU Robotics logo
extern const Image LOGO;       // boot splash
extern const Image LOGO_SMALL; // dashboard header badge

// Anti-aliased bitmap font: one 8-bit coverage bitmap per glyph, for ASCII 32-126
// followed by the degree sign (index 95).
struct Glyph {
    std::uint32_t offset; // into Font::bitmap
    std::uint8_t w, h;
    std::int8_t xoff, yoff; // bitmap top-left relative to the pen position / line top
    std::uint8_t advance;
};

struct Font {
    const Glyph *glyphs;
    const std::uint8_t *bitmap;
    int ascent;
    int lineHeight;
};

extern const Font FONT_SMALL;  // labels
extern const Font FONT_MEDIUM; // status text
extern const Font FONT_LARGE;  // temperatures and big numbers
extern const Font FONT_XLARGE; // splash title
