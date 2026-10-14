#include "stepper.hpp"

void Stepper::init()
{
    // Set the level before switching the pin to an output. Doing it the other
    // way round makes the pin briefly drive low, and a low on PUL is a step -
    // so the motor would twitch every time the Pico restarted.
    gpio_init(pin_pul_);
    gpio_put(pin_pul_, PUL_IDLE);
    gpio_set_dir(pin_pul_, GPIO_OUT);

    dir_positive_ = true;
    gpio_init(pin_dir_);
    gpio_put(pin_dir_, dir_level_for(spec_, dir_positive_));
    gpio_set_dir(pin_dir_, GPIO_OUT);
}

bool Stepper::pins_ok() const
{
    return gpio_get(pin_pul_) == PUL_IDLE &&
           gpio_get(pin_dir_) == dir_level_for(spec_, dir_positive_);
}

void Stepper::set_direction(bool positive)
{
    const bool reversing = (positive != dir_positive_);

    dir_positive_ = positive;
    gpio_put(pin_dir_, dir_level_for(spec_, positive));

    if (reversing) {
        sleep_ms(REVERSE_PAUSE_MS);
    }
    busy_wait_us(DIR_SETUP_US);
}

void Stepper::step()
{
    gpio_put(pin_pul_, PUL_PULSE);
    busy_wait_us(PULSE_LOW_US);
    gpio_put(pin_pul_, PUL_IDLE);

    pos_steps_ += dir_positive_ ? 1 : -1;
}
