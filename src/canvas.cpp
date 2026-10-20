#include "canvas.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

void Canvas::copyFrom(const std::uint32_t *frame) { std::memcpy(px, frame, sizeof(px)); }

void Canvas::blendPixel(int x, int y, std::uint32_t color, int alpha) {
    if (x < 0 || y < 0 || x >= W || y >= H || alpha <= 0) return;
    std::uint32_t &p = px[y * W + x];
    p = alpha >= 255 ? (color & 0xFFFFFF) : mixColor(p, color, alpha);
}

void Canvas::fillRect(int x0, int y0, int x1, int y1, std::uint32_t color, int alpha) {
    x0 = std::max(x0, 0), y0 = std::max(y0, 0);
    x1 = std::min(x1, W), y1 = std::min(y1, H);
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) blendPixel(x, y, color, alpha);
}

// Corner pixels get partial coverage based on their distance from the corner's
// arc, which gives smooth (anti-aliased) rounded edges.
void Canvas::fillRoundRect(int x0, int y0, int x1, int y1, int r, std::uint32_t color, int alpha) {
    r = std::min(r, std::min((x1 - x0) / 2, (y1 - y0) / 2));
    for (int y = std::max(y0, 0); y < std::min(y1, H); y++) {
        for (int x = std::max(x0, 0); x < std::min(x1, W); x++) {
            float cx = x < x0 + r ? x0 + r : (x >= x1 - r ? x1 - r : x + 0.5f);
            float cy = y < y0 + r ? y0 + r : (y >= y1 - r ? y1 - r : y + 0.5f);
            float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
            float cover = r + 0.5f - std::sqrt(dx * dx + dy * dy);
            if (cover <= 0) continue;
            blendPixel(x, y, color, cover >= 1 ? alpha : (int)(alpha * cover));
        }
    }
}

void Canvas::fillCircle(int cx, int cy, int r, std::uint32_t color, int alpha) {
    fillRoundRect(cx - r, cy - r, cx + r, cy + r, r, color, alpha);
}

void Canvas::card(int x0, int y0, int x1, int y1, int r, std::uint32_t fill, std::uint32_t border) {
    fillRoundRect(x0, y0, x1, y1, r, border);
    fillRoundRect(x0 + 1, y0 + 1, x1 - 1, y1 - 1, r - 1, fill);
}

void Canvas::image(const Image &img, int x, int y) {
    for (int iy = 0; iy < img.h; iy++) {
        for (int ix = 0; ix < img.w; ix++) {
            std::uint32_t c = img.palette[img.pixels[iy * img.w + ix]];
            blendPixel(x + ix, y + iy, c, c >> 24);
        }
    }
}

// Maps a byte of the string to a glyph index; the UTF-8 degree sign is C2 B0.
static int glyphIndex(const char *&s) {
    unsigned char c = (unsigned char)*s;
    if (c == 0xC2 && (unsigned char)s[1] == 0xB0) {
        s++;
        return 95;
    }
    if (c < 32 || c > 126) return '?' - 32;
    return c - 32;
}

int Canvas::textWidth(const Font &font, const char *text) const {
    int w = 0;
    for (const char *s = text; *s; s++) w += font.glyphs[glyphIndex(s)].advance;
    return w;
}

void Canvas::text(const Font &font, int x, int y, const char *text, std::uint32_t color, Align align) {
    if (align == CENTER) x -= textWidth(font, text) / 2;
    else if (align == RIGHT) x -= textWidth(font, text);

    for (const char *s = text; *s; s++) {
        const Glyph &g = font.glyphs[glyphIndex(s)];
        const std::uint8_t *bits = font.bitmap + g.offset;
        for (int gy = 0; gy < g.h; gy++)
            for (int gx = 0; gx < g.w; gx++) blendPixel(x + g.xoff + gx, y + g.yoff + gy, color, bits[gy * g.w + gx]);
        x += g.advance;
    }
}

void Canvas::present() { pros::screen::copy_area(0, 0, W - 1, H - 1, px, W); }
