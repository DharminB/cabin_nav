#ifndef CABIN_JOYPAD_H
#define CABIN_JOYPAD_H

#include <array>

namespace cabin {

struct Joypad
{
    /**
     * @brief Buttons on logitec F710 joypad
     *
     * | Index | Button label | Additional description |
     * | :---: | :----------: | :--------------------: |
     * |   0   |      X       | blue color             |
     * |   1   |      A       | green color            |
     * |   2   |      B       | red color              |
     * |   3   |      Y       | yellow color           |
     * |   4   |     LB       | Left button            |
     * |   5   |     RB       | Right button           |
     * |   6   |              |                        |
     * |   7   |              |                        |
     * |   8   |              |                        |
     * |   9   |              |                        |
     * |  10   |              |                        |
     * |  11   |              |                        |
     */
    std::array<int, 12> buttons;

    /**
     * @brief Axes on logitec F710 joypad
     *
     * | Index | Description               |
     * | :---: | :-----------------------: |
     * |   0   | Left joystick horizontal  |
     * |   1   | Left joystick vertical    |
     * |   2   | Right joystick horizontal |
     * |   3   | Right joystick vertical   |
     * |   4   | Left Trigger (LT)         |
     * |   5   | Right Trigger (RT)        |
     */
    std::array<float, 6> axes;

    friend std::ostream& operator << (std::ostream& out, const Joypad& joypad)
    {
        out << "buttons: [";
        for ( size_t i = 0; i < joypad.buttons.size(); i++ )
        {
            out << joypad.buttons[i] << ", ";
        }
        out << "]" << std::endl;

        out << "axes: [";
        for ( size_t i = 0; i < joypad.axes.size(); i++ )
        {
            out << joypad.axes[i] << ", ";
        }
        out << "]";
        return out;
    };
};

} // namespace cabin

#endif // CABIN_JOYPAD_H
