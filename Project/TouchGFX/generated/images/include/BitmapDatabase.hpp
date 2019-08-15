// Generated by imageconverter. Please, do not edit!

#ifndef BITMAPDATABASE_HPP
#define BITMAPDATABASE_HPP

#include <touchgfx/hal/Types.hpp>
#include <touchgfx/Bitmap.hpp>

const uint16_t BITMAP_ARROW_DX_ID = 0; // Size: 40x40 pixels
const uint16_t BITMAP_ARROW_SX_ID = 1; // Size: 40x40 pixels
const uint16_t BITMAP_START_ID = 2; // Size: 40x40 pixels
const uint16_t BITMAP_STOP_ID = 3; // Size: 40x40 pixels
const uint16_t BITMAP_BLACK_BACKGROUND_ID = 4; // Size: 20x20 pixels
const uint16_t BITMAP_BLUE_PROGRESSINDICATORS_BG_MEDIUM_CIRCLE_INDICATOR_BG_LINE_FULL_ID = 5; // Size: 104x104 pixels
const uint16_t BITMAP_BLUE_PROGRESSINDICATORS_FILL_MEDIUM_CIRCLE_INDICATOR_FILL_LINE_FULL_ID = 6; // Size: 104x104 pixels

namespace BitmapDatabase
{
#ifndef NO_USING_NAMESPACE_TOUCHGFX
using namespace touchgfx;
#endif

class BitmapData;
const touchgfx::Bitmap::BitmapData* getInstance();
uint16_t getInstanceSize();
}

#endif