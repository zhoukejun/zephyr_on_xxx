#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS  50

/* The devicetree node identifier for the "led_red" alias. */
#define LED_RED_NODE DT_ALIAS(led0)
#define LED_GREEN_NODE DT_ALIAS(led1)
#define LED_BLUE_NODE DT_ALIAS(led2)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

void led_test(void)
{
	int ret;

	//-------------------------
	if (!gpio_is_ready_dt(&led_red)) {
		printk("led_red is not ready yet!");
		return;
	}

	ret = gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printk("led_red config failed!");
		return;
	}

	//-------------------------
	if (!gpio_is_ready_dt(&led_green)) {
		printk("led_green is not ready yet!");
		return;
	}

	ret = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printk("led_green config failed!");
		return;
	}

	//-------------------------
	if (!gpio_is_ready_dt(&led_blue)) {
		printk("led_blue is not ready yet!");
		return;
	}

	ret = gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printk("led_blue config failed!");
		return;
	}

	//-------------------------
	while (1) {
		ret = gpio_pin_toggle_dt(&led_red);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);

		ret = gpio_pin_toggle_dt(&led_green);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);

		ret = gpio_pin_toggle_dt(&led_blue);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);
	}
}

