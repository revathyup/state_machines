#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/util/queue.h"

#define LED1 0
#define LED2 1
#define LED3 2
#define LED4 3

#define BUTTON1 20
#define BUTTON2 21
#define BUTTON3 22

#define BUTTON_DEBOUNCE_DELAY 200 // milliseconds

typedef void (*state_func_t)(void);

typedef struct {
    uint8_t id;
    state_func_t Enter;
    state_func_t Do;
    state_func_t Exit;
    uint32_t delay_ms;
} state_t;

typedef enum {
    b1_evt = 0,
    b2_evt = 1,
    b3_evt = 2,
    no_evt = 3
} event_t;

queue_t event_queue;
uint slice_num;

void button_isr(uint gpio, uint32_t events) {
    static absolute_time_t last_interrupt_time = {0};
    absolute_time_t current_time = get_absolute_time();
    int64_t time_diff = absolute_time_diff_us(last_interrupt_time, current_time);

    if (time_diff > BUTTON_DEBOUNCE_DELAY * 1000) {
        event_t evt;
        if (gpio == BUTTON1) {
            evt = b1_evt;
        } else if (gpio == BUTTON2) {
            evt = b2_evt;
        } else if (gpio == BUTTON3) {
            evt = b3_evt;
        } else {
            return;
        }
        queue_try_add(&event_queue, &evt);
        last_interrupt_time = current_time;
    }
}

void private_init() {
    gpio_init(LED1);
    gpio_init(LED2);
    gpio_init(LED3);
    gpio_init(LED4);
    gpio_set_dir(LED1, GPIO_OUT);
    gpio_set_dir(LED2, GPIO_OUT);
    gpio_set_dir(LED3, GPIO_OUT);
    gpio_set_dir(LED4, GPIO_OUT);

    gpio_init(BUTTON1);
    gpio_init(BUTTON2);
    gpio_init(BUTTON3);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_set_dir(BUTTON2, GPIO_IN);
    gpio_set_dir(BUTTON3, GPIO_IN);
    gpio_pull_up(BUTTON1);
    gpio_pull_up(BUTTON2);
    gpio_pull_up(BUTTON3);

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON3, GPIO_IRQ_EDGE_FALL, true);

    queue_init(&event_queue, sizeof(event_t), 10);
}

event_t get_event(void) {
    event_t evt;
    if (queue_try_remove(&event_queue, &evt)) {
        return evt;
    }
    return no_evt;
}

void leds_off() {
    gpio_put(LED1, 0);
    gpio_put(LED2, 0);
    gpio_put(LED3, 0);
    gpio_put(LED4, 0);
}

void leds_on() {
    gpio_put(LED1, 1);
    gpio_put(LED2, 1);
    gpio_put(LED3, 1);
    gpio_put(LED4, 1);
}

void do_state_0(void) {
    static int current_led = LED1;
    static int previous_led = -1;

    if (previous_led != -1) {
        gpio_put(previous_led, 0);
    }

    gpio_put(current_led, 1);

    previous_led = current_led;
    current_led = (current_led + 1 > LED4) ? LED1 : current_led + 1;
}

void do_state_1(void) {
    static bool led_state = false;
    led_state = !led_state;
    gpio_put(LED1, led_state);
    gpio_put(LED2, led_state);
    gpio_put(LED3, led_state);
    gpio_put(LED4, led_state);
}

void do_state_2(void) {
    static int current_led = LED4;
    static int previous_led = -1;

    if (previous_led != -1) {
        gpio_put(previous_led, 0);
    }

    gpio_put(current_led, 1);

    previous_led = current_led;
    current_led = (current_led - 1 < LED1) ? LED4 : current_led - 1;
}

void enter_state_3(void) {
    // Initialize PWM for LED1
    gpio_set_function(LED1, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(LED1);
    pwm_set_wrap(slice_num, 65535);
    pwm_set_gpio_level(LED1, 0);
    pwm_set_enabled(slice_num, true);
}

void do_state_3(void) {
    static int direction = 1;
    static uint16_t level = 0;

    pwm_set_gpio_level(LED1, level);
    level += direction * 256;

    if (level == 0 || level == 65535) {
        direction = -direction;
    }
}

void exit_state_3(void) {
    pwm_set_enabled(slice_num, false);
    gpio_set_function(LED1, GPIO_FUNC_SIO);
    gpio_put(LED1, 0);
}

const state_t state0 = {0, leds_on, do_state_0, leds_off, 500};
const state_t state1 = {1, leds_on, do_state_1, leds_off, 1000};
const state_t state2 = {2, leds_on, do_state_2, leds_off, 250};
const state_t state3 = {3, enter_state_3, do_state_3, exit_state_3, 10};

const state_t* state_table[4][4] = {
    {&state1, &state2, &state3, &state0},
    {&state0, &state2, &state3, &state1},
    {&state0, &state1, &state3, &state2},
    {&state0, &state0, &state0, &state3}
};

int main() {
    private_init();

    const state_t* current_state = &state0;
    event_t evt = no_evt;

    current_state->Enter();

    while (true) {
        current_state->Do();
        sleep_ms(current_state->delay_ms);
        evt = get_event();
        if (evt != no_evt) {
            current_state->Exit();
            current_state = state_table[current_state->id][evt];
            current_state->Enter();
        }
    }
}
