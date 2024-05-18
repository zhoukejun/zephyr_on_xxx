
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

//---------------
/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define USER_KEY_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(USER_KEY_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined!"
#endif
static const struct gpio_dt_spec user_key = GPIO_DT_SPEC_GET_OR(USER_KEY_NODE, gpios, {0});

//---------------
/* The devicetree node identifier for the "led0" alias. */
#define USER_LED_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec user_led = GPIO_DT_SPEC_GET_OR(USER_LED_NODE, gpios, {0});

static struct gpio_callback user_key_cb_data;


//---------------
void user_key_pressed(const struct device *dev, struct  gpio_callback *cb,
		    uint32_t pins)
{
	printk("user_key pressed at %" PRIu32 "\n", k_cycle_get_32());
}


#ifdef CONFIG_ARCH_POSIX
#define RETURN_FROM_MAIN(exit_code) posix_exit_main(exit_code)
#else
#define RETURN_FROM_MAIN(exit_code) return
#endif

#if 0
#define SLEEPTIME 3000
void main(void)
{
	while (1) {
		printk("%s %s %s %s\n", CONFIG_ARCH, CONFIG_BOARD, __DATE__, __TIME__);

		k_msleep(SLEEPTIME);

	}

	RETURN_FROM_MAIN(0);
}

#endif


#if 1

/*
 * The hello world demo has two threads that utilize semaphores and sleeping
 * to take turns printing a greeting message at a controlled rate. The demo
 * shows both the static and dynamic approaches for spawning a thread; a real
 * world application would likely use the static approach for both threads.
 */

#define PIN_THREADS (IS_ENABLED(CONFIG_SMP) && IS_ENABLED(CONFIG_SCHED_CPU_MASK))

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay between greetings (in ms) */
#define SLEEPTIME 500


/*
 * @param my_name      thread identification string
 * @param my_sem       thread's own semaphore
 * @param other_sem    other thread's semaphore
 */
void helloLoop(const char *my_name,
	       struct k_sem *my_sem, struct k_sem *other_sem)
{
	int ret;

	const char *tname;
	uint8_t cpu;
	struct k_thread *current_thread;

	while (1) {
		/* take my semaphore */
		k_sem_take(my_sem, K_FOREVER);

		current_thread = k_current_get();
		tname = k_thread_name_get(current_thread);
#if CONFIG_SMP
		cpu = arch_curr_cpu()->id;
#else
		cpu = 0;
#endif
		/* say "hello" */
		if (tname == NULL) {
		//	printk("%s: Hello World from cpu %d on %s!\n",
		//		my_name, cpu, CONFIG_BOARD);

			//-------
			ret = gpio_pin_toggle_dt(&user_led);
			if (ret < 0) {
				printk("%s: toggled user_led failed!\n", my_name);
				return;
			} else {
		//		printk("%s: user_led toggle!\n", my_name);
			}

		} else {
		//	printk("%s: Hello World from cpu %d on %s!\n",
		//		tname, cpu, CONFIG_BOARD);

			//-------
			ret = gpio_pin_toggle_dt(&user_led);
			if (ret < 0) {
				printk("%s: toggled user_led failed!\n", tname);
				return;
			} else {
		//		printk("%s: user_led toggle!\n", tname);
			}
		}

		if(gpio_pin_get_dt(&user_key))
			printk("user_ked pressed");

		/* wait a while, then let other thread have a turn */
//		k_busy_wait(500);
		k_msleep(SLEEPTIME);
		k_sem_give(other_sem);
	}
}

/* define semaphores */

K_SEM_DEFINE(threadA_sem, 1, 1);	/* starts off "available" */
K_SEM_DEFINE(threadB_sem, 0, 1);	/* starts off "not available" */


/* threadB is a dynamic thread that is spawned by threadA */

void threadB(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	/* invoke routine to ping-pong hello messages with threadA */
	helloLoop(__func__, &threadB_sem, &threadA_sem);
}

K_THREAD_STACK_DEFINE(threadA_stack_area, STACKSIZE);
static struct k_thread threadA_data;

K_THREAD_STACK_DEFINE(threadB_stack_area, STACKSIZE);
static struct k_thread threadB_data;

/* threadA is a static thread that is spawned automatically */

void threadA(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	/* invoke routine to ping-pong hello messages with threadB */
	helloLoop(__func__, &threadA_sem, &threadB_sem);
}

void main(void)
{
	int ret;

	//---user_led---
	if (!gpio_is_ready_dt(&user_led)) {
		printk("user_led is not ready yet!\n");
		return;
	}

	ret = gpio_pin_configure_dt(&user_led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printk("user_led config failed!\n");
		return;
	} else {
		printk("Set up user_led at %s pin %d\n", user_led.port->name, user_led.pin);
	}

	//---button---
	if(!gpio_is_ready_dt(&user_key)) {
		printk("Error: user_key device %s is not ready\n",
		    	user_key.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&user_key, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
			ret, user_key.port->name, user_key.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&user_key,
						GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, user_key.port->name, user_key.pin);
		return;
	}

	gpio_init_callback(&user_key_cb_data, user_key_pressed, BIT(user_key.pin));
	gpio_add_callback(user_key.port, &user_key_cb_data);
	printk("Set up user_key at %s pin %d\n", user_key.port->name, user_key.pin);

	//------------
	k_thread_create(&threadA_data, threadA_stack_area,
			K_THREAD_STACK_SIZEOF(threadA_stack_area),
			threadA, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&threadA_data, "thread_a");
#if PIN_THREADS
	if (arch_num_cpus() > 1) {
		k_thread_cpu_pin(&threadA_data, 0);
	}
#endif

	k_thread_create(&threadB_data, threadB_stack_area,
			K_THREAD_STACK_SIZEOF(threadB_stack_area),
			threadB, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&threadB_data, "thread_b");
#if PIN_THREADS
	if (arch_num_cpus() > 1) {
		k_thread_cpu_pin(&threadB_data, 1);
	}
#endif

	k_thread_start(&threadA_data);
	k_thread_start(&threadB_data);

	RETURN_FROM_MAIN(0);
}
#endif
