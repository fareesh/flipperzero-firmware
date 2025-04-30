// insired by
// https://github.com/esphome/esphome/blob/ac0d921413c3884752193fe568fa82853f0f99e9/esphome/components/ultrasonic/ultrasonic_sensor.cpp

#include <furi.h>
#include <furi_hal_light.h>
#include <furi_hal_power.h>
#include <gui/gui.h>
#include <input/input.h>
#include <stdlib.h>
#include <gui/elements.h>
#include <string.h>
#include <inttypes.h>


typedef enum {
    EventTypeTick,
    EventTypeKey,
} EventType;

typedef struct {
    EventType type;
    InputEvent input;
} PluginEvent;

typedef struct {
    bool        have_5v;
    bool        measurement_made;
    uint32_t    echo_ms;            // echo pulse length in ms (1 tick resolution)
    float       distance_m;         // calculated distance in metres
    FuriMutex*  mutex;              // NEW: protects this struct
} PluginState;

static inline float ms_to_m(uint32_t ms) {
    const float speed = 343.0f;                // m / s @ 20 °C
    return (ms * 0.001f * speed) * 0.5f;       // go-&-return
}

static void render_cb(Canvas* canvas, void* ctx) {
    PluginState* st = ctx;
    if(furi_mutex_acquire(st->mutex, 25) != FuriStatusOk) return;

    canvas_set_font(canvas, FontPrimary);
    elements_multiline_text_aligned(
        canvas, 64, 2, AlignCenter, AlignTop,
        "HC-SR04 Ultrasonic\nDistance Sensor");

    canvas_set_font(canvas, FontSecondary);
    if(!st->have_5v) {
        elements_multiline_text_aligned(
            canvas, 4, 28, AlignLeft, AlignTop,
            "5 V on GPIO must be\nenabled, or USB must\nbe connected.");
    } else if(!st->measurement_made) {
        elements_multiline_text_aligned(
            canvas, 64, 28, AlignCenter, AlignTop,
            "Press centre button\nto measure");
    } else {
        elements_multiline_text_aligned(canvas, 4, 28, AlignLeft, AlignTop, "Readout:");

        FuriString* s = furi_string_alloc();
        furi_string_printf(s, "Echo: %" PRIu32 " ms", (uint32_t)st->echo_ms);

        canvas_draw_str_aligned(canvas, 8, 38, AlignLeft, AlignTop, furi_string_get_cstr(s));

        furi_string_printf(s, "Distance: %.2f m", (double)st->distance_m);

        canvas_draw_str_aligned(canvas, 8, 48, AlignLeft, AlignTop, furi_string_get_cstr(s));
        furi_string_free(s);
    }

    furi_mutex_release(st->mutex);
}

static void input_cb(InputEvent* ev, void* ctx) {
    FuriMessageQueue* q = ctx;
    PluginEvent e      = {.type = EventTypeKey, .input = *ev};
    furi_message_queue_put(q, &e, FuriWaitForever);
}

static void hc_sr04_measure(PluginState* st) {
    if(!st->have_5v) {
        st->have_5v =
            (furi_hal_power_is_otg_enabled() || furi_hal_power_is_charging());
        if(!st->have_5v) return;
    }

    furi_hal_light_set(LightRed, 0xFF);

    const uint32_t timeout_ms = 2000;

    furi_hal_gpio_write(&gpio_usart_tx, false);  // Trig low
    furi_hal_gpio_init(
        &gpio_usart_tx, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);

    furi_hal_gpio_init(
        &gpio_usart_rx, GpioModeInput, GpioPullNo, GpioSpeedVeryHigh);

    furi_hal_gpio_write(&gpio_usart_tx, true);
    furi_delay_us(10);
    furi_hal_gpio_write(&gpio_usart_tx, false);

    const uint32_t start = furi_get_tick();

    while((furi_get_tick() - start) < timeout_ms && furi_hal_gpio_read(&gpio_usart_rx))
        ;
    while((furi_get_tick() - start) < timeout_ms && !furi_hal_gpio_read(&gpio_usart_rx))
        ;

    const uint32_t pulse_start = furi_get_tick();

    while((furi_get_tick() - start) < timeout_ms && furi_hal_gpio_read(&gpio_usart_rx))
        ;

    const uint32_t pulse_end = furi_get_tick();

    st->echo_ms         = pulse_end - pulse_start;
    st->distance_m      = ms_to_m(st->echo_ms);
    st->measurement_made = true;

    furi_hal_light_set(LightRed, 0x00);
}

int32_t hc_sr04_app(void* p) {
    UNUSED(p);
    srand(furi_get_tick());

    PluginState state = {
        .have_5v          = furi_hal_power_is_otg_enabled() || furi_hal_power_is_charging(),
        .measurement_made = false,
        .echo_ms          = 0,
        .distance_m       = 0.0f,
        .mutex            = furi_mutex_alloc(FuriMutexTypeRecursive),
    };
    if(!state.mutex) return 255;

    FuriMessageQueue* queue = furi_message_queue_alloc(8, sizeof(PluginEvent));

    ViewPort* vp = view_port_alloc();
    view_port_draw_callback_set(vp, render_cb, &state);
    view_port_input_callback_set(vp, input_cb, queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, vp, GuiLayerFullscreen);

    PluginEvent event;
    for(bool running = true; running;) {
        const FuriStatus s = furi_message_queue_get(queue, &event, 100);

        furi_mutex_acquire(state.mutex, FuriWaitForever);

        if(s == FuriStatusOk && event.type == EventTypeKey &&
           event.input.type == InputTypePress) {
            switch(event.input.key) {
            case InputKeyOk:
                hc_sr04_measure(&state);
                break;
            case InputKeyBack:
                running = false;
                break;
            default:;
            }
        }

        furi_mutex_release(state.mutex);
        view_port_update(vp);
    }

    view_port_enabled_set(vp, false);
    gui_remove_view_port(gui, vp);
    furi_record_close(RECORD_GUI);
    view_port_free(vp);

    furi_message_queue_free(queue);
    furi_mutex_free(state.mutex);

    return 0;
}
