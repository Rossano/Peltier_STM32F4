/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/mainscreen_screen/MainScreenViewBase.hpp>
#include "BitmapDatabase.hpp"
#include <touchgfx/Color.hpp>
#include <texts/TextKeysAndLanguages.hpp>

MainScreenViewBase::MainScreenViewBase()
{
    CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    PWMProgress.setXY(68, 200);
    PWMProgress.setProgressIndicatorPosition(0, 0, 104, 104);
    PWMProgress.setRange(0, 100);
    PWMProgress.setCenter(52, 52);
    PWMProgress.setRadius(50);
    PWMProgress.setLineWidth(0);
    PWMProgress.setStartEndAngle(-270, 90);
    PWMProgress.setBackground(Bitmap(BITMAP_BLUE_PROGRESSINDICATORS_BG_MEDIUM_CIRCLE_INDICATOR_BG_LINE_FULL_ID));
    PWMProgressPainter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
    PWMProgress.setPainter(PWMProgressPainter);
    PWMProgress.setValue(33);

    background.setBitmap(Bitmap(BITMAP_BLACK_BACKGROUND_ID));
    background.setPosition(0, 0, 240, 320);
    background.setScalingAlgorithm(ScalableImage::NEAREST_NEIGHBOR);

    PeltierTempText.setXY(205, 10);
    PeltierTempText.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
    PeltierTempText.setLinespacing(0);
    PeltierTempText.setRotation(TEXT_ROTATE_90);
    PeltierTempText.setTypedText(TypedText(T_TEMPPELTIER_TXT));

    ExtTempText.setXY(161, 10);
    ExtTempText.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
    ExtTempText.setLinespacing(0);
    ExtTempText.setRotation(TEXT_ROTATE_90);
    ExtTempText.setTypedText(TypedText(T_TEMPEXT_TXT));

    PeltierTemp.setXY(205, 70);
    PeltierTemp.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
    PeltierTemp.setLinespacing(0);
    PeltierTemp.setRotation(TEXT_ROTATE_90);
    PeltierTempBuffer[0] = 0;
    PeltierTemp.setWildcard(PeltierTempBuffer);
    PeltierTemp.resizeToCurrentText();
    PeltierTemp.setTypedText(TypedText(T_TEMPPELTIER));

    ExtTemp.setXY(161, 70);
    ExtTemp.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
    ExtTemp.setLinespacing(0);
    ExtTemp.setRotation(TEXT_ROTATE_90);
    ExtTempBuffer[0] = 0;
    ExtTemp.setWildcard(ExtTempBuffer);
    ExtTemp.resizeToCurrentText();
    ExtTemp.setTypedText(TypedText(T_TEMPEXT));

    PWMText.setXY(109, 10);
    PWMText.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
    PWMText.setLinespacing(0);
    PWMText.setRotation(TEXT_ROTATE_90);
    PWMTextBuffer[0] = 0;
    PWMText.setWildcard(PWMTextBuffer);
    PWMText.resizeToCurrentText();
    PWMText.setTypedText(TypedText(T_PWM_TXT));

    PWMDownButton.setXY(29, 15);
    PWMDownButton.setBitmap(Bitmap(BITMAP_ARROW_SX_ID));

    StartButton.setXY(60, 62);
    StartButton.setBitmap(Bitmap(BITMAP_START_ID));

    StopButton.setXY(9, 62);
    StopButton.setBitmap(Bitmap(BITMAP_STOP_ID));

    PWMUpButton.setXY(29, 109);
    PWMUpButton.setBitmap(Bitmap(BITMAP_ARROW_DX_ID));

    add(PWMProgress);
    add(background);
    add(PeltierTempText);
    add(ExtTempText);
    add(PeltierTemp);
    add(ExtTemp);
    add(PWMText);
    add(PWMDownButton);
    add(StartButton);
    add(StopButton);
    add(PWMUpButton);
}

void MainScreenViewBase::setupScreen()
{

}
