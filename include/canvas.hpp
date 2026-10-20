#pragma once
#include "screen_assets.hpp"
#include <cstdint>

// Off-screen 480x240 frame for the brain screen. Everything is drawn here first
// (with alpha blending and anti-aliased text), then present() sends the finished
// frame to the screen in a single copy -- no half-drawn frames, so no flicker.
class Canvas {
  public:
    static const int W = 480;
    static const int H = 240;

    enum Align { LEFT, CENTER, RIGHT };

    // Coordinates are x0/y0 inclusive, x1/y1 exclusive. alpha: 255 = opaque.
    void copyFrom(const std::uint32_t *frame);
    void fillRect(int x0, int y0, int x1, int y1, std::uint32_t color, int alpha = 255);
    void fillRoundRect(int x0, int y0, int x1, int y1, int r, std::uint32_t color, int alpha = 255);
    void fillCircle(int cx, int cy, int r, std::uint32_t color, int alpha = 255);
    // Filled rounded rect with a 1px border in a different color.
    void card(int x0, int y0, int x1, int y1, int r, std::uint32_t fill, std::uint32_t border);
    // Blends a palettized image with its top-left corner at (x, y).
    void image(const Image &img, int x, int y);

    // y is the top of the text line. Handles the UTF-8 degree sign.
    int textWidth(const Font &font, const char *text) const;
    void text(const Font &font, int x, int y, const char *text, std::uint32_t color, Align align = LEFT);

    void present();

    std::uint32_t *pixels() { return px; }
    const std::uint32_t *pixels() const { return px; }

  private:
    void blendPixel(int x, int y, std::uint32_t color, int alpha);
    std::uint32_t px[W * H];
};

// 0xRRGGBB mix of a toward b, t = 0..255
inline std::uint32_t mixColor(std::uint32_t a, std::uint32_t b, int t) {
    std::uint32_t r = (((a >> 16) & 0xFF) * (255 - t) + ((b >> 16) & 0xFF) * t) / 255;
    std::uint32_t g = (((a >> 8) & 0xFF) * (255 - t) + ((b >> 8) & 0xFF) * t) / 255;
    std::uint32_t bl = ((a & 0xFF) * (255 - t) + (b & 0xFF) * t) / 255;
    return (r << 16) | (g << 8) | bl;
}
