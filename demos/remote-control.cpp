#include <chrono>
#include <thread>
#include <stdexcept>

#include <ev3dev.h>
#include <iostream>


/*
-----------------------------------------------------------------------------
Copyright (c) 2015 Denis Demidov <dennis.demidov@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------

This demo shows how to remote control an Explor3r robot with
touch sensor attachment.

Red buttons control left motor, blue buttons control right motor.
Leds are used to indicate movement direction.
Whenever an obstacle is bumped, robot backs away and apologises.
*/

namespace ev3 = ev3dev;

std::ostream& operator<<(std::ostream &os, const std::set<std::string> &ss) {
    os << "[ ";
    for(const auto &s : ss) os << s << " ";
    return os << "]";
}

//---------------------------------------------------------------------------
void precondition(bool cond, const std::string &msg) {
    if (!cond) throw std::runtime_error(msg);
}

//---------------------------------------------------------------------------
template <class Motor, class Leds>
std::function<void(bool)> roll(Motor &motor, Leds &leds, int dir) {
    return [&motor, &leds, dir](bool state) {
        if (state) {
            motor.set_speed_sp(900 * dir).run_forever();
            ev3::led::set_color(leds, dir > 0 ? ev3::led::green : ev3::led::red);
        } else {
            motor.set_stop_action("brake").stop();
            for(auto led : leds) led->off();
        }
    };
}

template <class Motor, class Leds>
std::function<void(bool)> roll(Motor &motor1, Motor &motor2, int distance_rot, int speed, int dir1, int dir2, Leds &leds, int delay_ms) {
    return [&motor1, &motor2, dir1, dir2, distance_rot, speed, &leds, delay_ms](bool state) {
        if (state) {
            motor1.set_position_sp(distance_rot * dir1).set_speed_sp(speed).run_to_rel_pos();
            motor2.set_position_sp(distance_rot * dir2).set_speed_sp(speed).run_to_rel_pos();
            ev3::led::set_color(leds, dir1 > 0 ? ev3::led::green : ev3::led::red);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        } else {
            motor1.set_stop_action("brake").stop();
            motor2.set_stop_action("brake").stop();
            for(auto led : leds) led->off();
        }
    };
}

enum mapColors
{
    BLACK,
    BLUE,
    GREEN,
    PURPLE,
    YELLOW,
    RED,
    PINK,
    NO_COLOR,
    NUM = NO_COLOR
};
uint8_t const mapRows{9};
uint8_t const mapColumns{4};
static const int colorMap[mapRows][mapColumns]
{
    mapColors::BLUE,    mapColors::PURPLE,  mapColors::RED,     mapColors::BLUE,
    mapColors::PINK,    mapColors::BLACK,   mapColors::PINK,    mapColors::BLACK,
    mapColors::YELLOW,  mapColors::BLACK,   mapColors::BLACK,   mapColors::RED,
    mapColors::PURPLE,  mapColors::BLUE,    mapColors::YELLOW,  mapColors::RED,
    mapColors::BLACK,   mapColors::BLACK,   mapColors::PURPLE,  mapColors::PURPLE,
    mapColors::PINK,    mapColors::YELLOW,  mapColors::BLACK,   mapColors::PINK,
    mapColors::PURPLE,  mapColors::BLACK,   mapColors::RED,     mapColors::PURPLE,
    mapColors::GREEN,   mapColors::PURPLE,  mapColors::BLUE,    mapColors::BLACK,
    mapColors::BLUE,    mapColors::BLACK,   mapColors::GREEN,   mapColors::BLUE
};

bool numberInRange(uint16_t pNumber, uint16_t lowValue, uint16_t highValue, uint8_t buffer = 0)
{
    bool inRange = false;
    inRange = (pNumber >= (lowValue - buffer)) && (pNumber <= (highValue + buffer));
//    std::cout << "Low Value: " << lowValue - buffer << " Num: " << pNumber << " High value: " << highValue + buffer << " inRange: " << inRange << std::endl;
    return inRange;
}

static constexpr uint8_t idxBlack    {0};
static constexpr uint8_t idxBlue     {1};
static constexpr uint8_t idxGreen    {2};
static constexpr uint8_t idxPurple   {3};
static constexpr uint8_t idxYellow   {4};
static constexpr uint8_t idxRed      {5};
static constexpr uint8_t idxPink     {6};

static constexpr int numColors          {7};
static constexpr int rgbHighLowIndices  {6};

bool rgbMatch(std::tuple<int, int, int> rgb, uint8_t const pColorIndex, uint8_t const buffer = 0)
{
//    std::cout << " Color: " << pColorIndex;
//    printf("Color: %2d ", pColorIndex);
    static const std::array<std::array<uint16_t, rgbHighLowIndices>, numColors> clrTable
    {
        20, 22, 22, 23, 29,  31,
        43, 48,155,166,260, 290,
        63, 71,156,175, 76,  86,
        89,106, 54, 73,148, 168,
        261,272,239,257,260, 264,
        250,257, 32, 35, 48,  51,
        285,293,224,248,382, 401
    };

    static constexpr uint8_t idxRLow     {0};
    static constexpr uint8_t idxRHigh    {1};
    static constexpr uint8_t idxGLow     {2};
    static constexpr uint8_t idxGHigh    {3};
    static constexpr uint8_t idxBLow     {4};
    static constexpr uint8_t idxBHigh    {5};

    uint16_t r = std::get<0>(rgb);
    uint16_t g = std::get<1>(rgb);
    uint16_t b = std::get<2>(rgb);

    bool match = false;
    match = (
            numberInRange(r, clrTable[pColorIndex][idxRLow], clrTable[pColorIndex][idxRHigh], buffer) &&
            numberInRange(g, clrTable[pColorIndex][idxGLow], clrTable[pColorIndex][idxGHigh], buffer) &&
            numberInRange(b, clrTable[pColorIndex][idxBLow], clrTable[pColorIndex][idxBHigh], buffer)
            );

//    std::cout << " Match: " << match << " Color: " << pColorIndex << " RGB Column: " << idxRLow << ", " << idxRHigh << ", " << idxGLow << ", " << idxGHigh << ", " << idxBLow << ", " << idxBHigh <<
//    std::cout << " Match: " << match << " Color: " << pColorIndex << std::endl;
//    std::cout << " Match: " << match << std::endl;
    return match;
}

mapColors getColor(std::tuple<int, int, int> rgb)
{
    static mapColors colorDetected = mapColors::NO_COLOR;

    static const int bffr{50};

    if      (rgbMatch(rgb, 0, bffr)) {colorDetected = mapColors::BLACK;}
    else if (rgbMatch(rgb, 1, bffr)) {colorDetected = mapColors::BLUE;}
    else if (rgbMatch(rgb, 2, bffr)) {colorDetected = mapColors::GREEN;}
    else if (rgbMatch(rgb, 3, bffr)) {colorDetected = mapColors::PURPLE;}
    else if (rgbMatch(rgb, 4, bffr)) {colorDetected = mapColors::YELLOW;}
    else if (rgbMatch(rgb, 5, bffr)) {colorDetected = mapColors::RED;}
    else if (rgbMatch(rgb, 6, bffr)) {colorDetected = mapColors::PINK;}
    else                                              {colorDetected = mapColors::NO_COLOR;}

    return colorDetected;
}

std::string getColorString(uint8_t colorIndex)
{
//    cout << "Get Color String Called\n";
    std::string colorDetected;
    switch (colorIndex)
    {
        case (mapColors::BLACK):
        {
            colorDetected = "Black";
            break;
        }
        case (mapColors::BLUE):
        {
            colorDetected = "Blue";
            break;
        }
        case (mapColors::GREEN):
        {
            colorDetected = "Green";
            break;
        }
        case (mapColors::PURPLE):
        {
            colorDetected = "Purple";
            break;
        }
        case (mapColors::YELLOW):
        {
            colorDetected = "Yellow";
            break;
        }
        case (mapColors::RED):
        {
            colorDetected = "Red";
            break;
        }
        case (mapColors::PINK):
        {
            colorDetected = "Pink";
            break;
        }
        case (mapColors::NO_COLOR):
        {
            colorDetected = "No color";
            break;
        }
        default:
        {
            colorDetected = "Error";
            break;
        }
    }

//    std::cout <<"Color Detected: " << colorDetected << "\n";

    return colorDetected;
}

const std::array<float, mapRows>& robotLocalize(mapColors currColor)
{
    static std::array<float, mapRows> mapLocalizationDistribution{};
    static const float uniformDistValue = (1.0 / mapRows);
    static const float measurementAccuracyLikelihood{0.9};
    static float normalizer{0};

    if(currColor == mapColors::NO_COLOR)
    {
        // reset distribution
        std::fill(mapLocalizationDistribution.begin(), mapLocalizationDistribution.end(), uniformDistValue);
        for (int i = 0; i < mapRows; ++i)
        {
            std::cout << "index: " << i << " probability: " << mapLocalizationDistribution[i] << std::endl;
        }
    } else {
        normalizer = 0;
        for (int i = 0; i < mapRows; ++i)
        {
            mapLocalizationDistribution[i] = ((currColor == colorMap[i][1]) ? (measurementAccuracyLikelihood * mapLocalizationDistribution[i]) : ((1 - measurementAccuracyLikelihood) * mapLocalizationDistribution[i]));
            normalizer += mapLocalizationDistribution[i];
//            std::cout << "index: " << i << " MapColor at index: " <<  getColorString(colorMap[i][1]) << "\t probability: " << mapLocalizationDistribution[i] << std::endl;
        }
        for (int i = 0; i < mapRows; ++i)
        {
            mapLocalizationDistribution[i] = mapLocalizationDistribution[i] / normalizer;
            std::cout << "index: " << i << " MapColor at index: " <<  getColorString(colorMap[i][1]) << "\t probability: " << mapLocalizationDistribution[i] << std::endl;
        }
    }
    return mapLocalizationDistribution;
}

//---------------------------------------------------------------------------
int main() {
    ev3::large_motor lmotor(ev3::OUTPUT_B);
    ev3::large_motor rmotor(ev3::OUTPUT_C);
    ev3::remote_control rc;
    ev3::touch_sensor   ts;
    ev3::color_sensor   cs;

    cs.set_mode(ev3::color_sensor::mode_rgb_raw);

    precondition(lmotor.connected(), "Motor on outB is not connected");
    precondition(rmotor.connected(), "Motor on outC is not connected");
    precondition(rc.connected(),     "Infrared sensor is not connected");

    rc.on_red_up    = roll(lmotor, rmotor, 400, 450,  1,   1, ev3::led::left, 2000);
    rc.on_red_down  = roll(lmotor, rmotor, 400, 450, -1,  -1, ev3::led::left, 2000);
    rc.on_blue_up   = roll(lmotor, rmotor, 10, 450, -1,  1, ev3::led::right, 2000);
    rc.on_blue_down = roll(lmotor, rmotor, 10, 450, 1, -1, ev3::led::right, 2000);

    std::cout << "Print Map: \n";
    for(int i = 0; i < mapRows; ++i)
    {
        for (int j = 0; j < mapColumns; ++j) {
            std::cout << getColorString(colorMap[i][j]) << "\t";
        }
        std::cout << std::endl;
    }

    //initialize localization distribution
    mapColors colorRead = mapColors::NO_COLOR;
    getColorString(colorRead);
    auto robotPositionDistribution = robotLocalize(colorRead);

    // read initial position
    colorRead = getColor(cs.raw());
    std::cout << "Initial color read: " << getColorString(colorRead) << "\n";
    robotPositionDistribution = robotLocalize(colorRead);

    // Enter event processing loop
    while (!ev3::button::enter.pressed()) {
        if(rc.process())
        {
            colorRead = getColor(cs.raw());
            std::cout << "Color Detected: " << getColorString(colorRead) << "\n";
            robotPositionDistribution = robotLocalize(colorRead);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
