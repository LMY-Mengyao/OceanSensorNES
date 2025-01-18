#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "max86177.h"
#include <stdio.h>

/* Delay interval in milliseconds */
#define SLEEP_MS 1000
/* Devicetree aliases */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define SW3_NODE  DT_ALIAS(sw3)
/* Node for the peripheral enable pin (originally zephyr_user) */
#define PERIPHERAL_ENABLE_NODE DT_PATH(zephyr_user)
/* Node for the peripheral enable pin (originally zephyr_user) */
#define AFE_TRIG_NODE DT_PATH(zephyr_user)
#define AFE_INT1_NODE DT_PATH(zephyr_user)
#define AFE_INT2_NODE DT_PATH(zephyr_user)


/* Define a variable of type static struct gpio_callback */
static struct gpio_callback afe_int1_cb_data;
static const struct gpio_dt_spec afe_trig = GPIO_DT_SPEC_GET(AFE_TRIG_NODE, afe_trig_gpios);
static const struct gpio_dt_spec afe_int1 = GPIO_DT_SPEC_GET(AFE_INT1_NODE, afe_int1_gpios);
static const struct gpio_dt_spec afe_int2 = GPIO_DT_SPEC_GET(AFE_INT2_NODE, afe_int2_gpios);

/* GPIO device structures derived from devicetree */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec sw3  = GPIO_DT_SPEC_GET(SW3_NODE,  gpios);
static const struct gpio_dt_spec peripheral_enable =
    GPIO_DT_SPEC_GET(PERIPHERAL_ENABLE_NODE, peripheral_enable_gpios);

/*  Define the callback function */
void afe_int1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	//gpio_pin_toggle_dt(&led);
   OS67_read_fifo();
}
/**
 * @brief Read the current state of the button.
 *
 * @return 0 if the button is pressed (logic low), 1 if the button is not pressed (logic high).
 */
int oceansensor_read_btn(void)
{
    return gpio_pin_get_dt(&sw3);
}

/**
 * @brief Control an LED based on the button state.
 *
 * If the button is pressed (logic low), turn on the LED (set it low if active-low).
 * Otherwise, turn off the LED (set it high if active-low).
 *
 * @param button Pointer to the button's gpio_dt_spec structure.
 * @param led    Pointer to the LED's gpio_dt_spec structure.
 */
void button_led_control(const struct gpio_dt_spec *button,
                        const struct gpio_dt_spec *led)
{
    int btn_state = gpio_pin_get_dt(button);

    if (btn_state == 0) {
        /* Button is pressed (low) */
        gpio_pin_set_dt(led, 0);  /* Turn LED on (active-low) */
    } else {
        /* Button is not pressed (high) */
        gpio_pin_set_dt(led, 1);  /* Turn LED off (active-low) */
    }
}

/**
 * @brief Initialize the LED and button GPIO pins.
 *
 * @return 0 if successful, otherwise a nonzero error code.
 */
int oceansensor_led_sw_init(void)
{
    int err = 0;

    /* Verify that the GPIO devices are ready */
    err |= !gpio_is_ready_dt(&led0);
    err |= !gpio_is_ready_dt(&led1);
    err |= !gpio_is_ready_dt(&sw3);

    if (err) {
        return err;
    }

    /* Configure the LED pins as outputs (initially inactive / off) */
    err |= gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    err |= gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

    /* Configure the button pin as an input with pull-up */
    err |= gpio_pin_configure_dt(&sw3, GPIO_INPUT | GPIO_PULL_UP);

    if (err) {
        return err;
    }

    return 0;
}
void OS67_sensor_init(void)
{
    int ret;

    /* Check if the device controlling the peripheral enable pin is ready */
    if (!device_is_ready(peripheral_enable.port)) {
        printk("GPIO device for peripheral_enable not ready.\n");
        return;
    }

    /* Configure peripheral_enable pin as output, initially set to inactive (low) */
    gpio_pin_configure_dt(&peripheral_enable, GPIO_OUTPUT_INACTIVE);

    /* Drive the pin high to enable the peripheral */
    gpio_pin_set_dt(&peripheral_enable, 1);

    /* Configure the interrupt 1 pin */
    gpio_pin_configure_dt(&afe_int1, GPIO_INPUT);

    ret = gpio_pin_interrupt_configure_dt(&afe_int1, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        printk("Failed to configure interrupt on afe_int1 pin.\n");
        return;
    }

    /* Initialize the static struct gpio_callback variable */
    gpio_init_callback(&afe_int1_cb_data, afe_int1_isr, BIT(afe_int1.pin));

    /* Add the callback function by calling gpio_add_callback() */
    gpio_add_callback(afe_int1.port, &afe_int1_cb_data);

    /* Perform I2C initialization for the sensor */
    os67_sensor_i2c_init();
    k_msleep(50);

    /* Enable the OS67 sensor */
    OS67_enable();
    k_msleep(50);
    printk("OS67 sensor initialization complete.\n");
}

int main(void)
{
    /* Initialize the LEDs and the button */
    if (oceansensor_led_sw_init() != 0) {
        printk("Failed to initialize GPIO\n");
        return -1;
    }
    /* Initialize the OS67 sensor */
    OS67_sensor_init();
    k_msleep(SLEEP_MS);
    k_msleep(SLEEP_MS);
    OS67_read_fifo();
    while (1) {
        /* Update LED1 based on the sw3 button state */
       button_led_control(&sw3, &led1);
        /* Delay to avoid excessive polling */
        k_msleep(SLEEP_MS);
    }
    return 0;
}
