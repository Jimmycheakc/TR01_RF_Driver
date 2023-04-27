#include <stdio.h>
#include <stdlib.h>
#include "io_utils.h"

extern int interrupt_handler(int event, unsigned int offset, const struct timespec *timestamp, void *userdata);

void gpio_output_test(const char *device, int offset)
{
    int ret;

    printf("Start --> %s\n", __func__);
    printf("1. %s: output gpio value = 1\n", __func__);

    ret = io_utils_write_gpio(device, offset, 1, false, "gpio_testing", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
    if (ret < 0)
    {
        printf("Failed to write gpio chip %s offset %d with output 1 && bias disable.\n", device, offset);
    }
    else
    {
        printf("Passed.\n");
    }

    printf("2. %s: output gpio value = 0\n", __func__);
    ret = io_utils_write_gpio(device, offset, 0, false, "gpio_testing", GPIOD_CTXLESS_FLAG_BIAS_DISABLE);
    if (ret < 0)
    {
        printf("Failed to write gpio chip %s offset %d with output 0 && bias disable.\n", device, offset);
    }
    else
    {
        printf("Passed.\n");
    }

    printf("End --> Completed testing with %s\n", __func__);
}

void gpio_output_wait_test(const char *device, int offset)
{
    int ret;

    printf("Start --> %s\n", __func__);
    printf("1. %s: gpio value = 1\n", __func__);

    ret = io_utils_write_gpio_with_wait(device, offset, 1, false, "gpio_testing", GPIOD_CTXLESS_FLAG_BIAS_DISABLE, 20);
    if (ret < 0)
    {
        printf("Failed to write gpio chip %s offset %d with output 1 && bias disable && wait.\n", device, offset);
    }
    else
    {
        printf("Passed.\n");
    }

    printf("2. %s: gpio value = 0\n", __func__);
    ret = io_utils_write_gpio_with_wait(device, offset, 0, false, "gpio_testing", GPIOD_CTXLESS_FLAG_BIAS_DISABLE, 20);
    if (ret < 0)
    {
        printf("Failed to write gpio chip %s offset %d with output 0 && bias disable && wait.\n", device, offset);
    }
    else
    {
        printf("Passed.\n");
    }

    printf("End --> Completed testing with %s\n", __func__);
}

void gpio_input_test(const char *device, int offset)
{
    int ret;

    printf("Start --> %s\n", __func__);
    printf("1. %s: gpio input && active high && bias pull up\n", __func__);

    ret = io_utils_read_gpio(device, offset, false, "gpio_testing", GPIOD_CTXLESS_FLAG_BIAS_PULL_UP);
    if (ret < 0)
    {
        printf("Failed to read input gpio chip %s offset %d with active high && bias pull up.\n", device, offset);
    }
    else
    {
        printf("Passed.\n");
    }

    printf("2. %s: gpio input && active low && bias pull down\n", __func__);
    ret = io_utils_read_gpio(device, offset, true, "gpio_testing", GPIOD_CTXLESS_FLAG_BIAS_PULL_DOWN);
    if (ret < 0)
    {
        printf("Failed to read input gpio chip %s offset %d with active low && bias pull down.\n", device, offset);
    }
    else
    {
        printf("Passed.\n");
    }

    printf("End --> Completed testing with %s\n", __func__);
}

int interrupt_handler(int event, unsigned int offset, const struct timespec *timestamp, void *userdata)
{
    printf("%s\n", __func__);
    exit(0);

    return 0;
}

void gpio_irq_monitor_test(const char *device, int offset)
{
    int ret;

    printf("Start --> %s\n", __func__);
    printf("1. %s: Monitoring interrupt pin\n", __func__);

    pthread_t pthread;
    ret = io_utils_setup_interrupt(device, GPIOD_CTXLESS_EVENT_FALLING_EDGE,
                                    offset, false,
                                    "gpio_interrupt",
                                    &interrupt_handler,
                                    NULL, GPIOD_CTXLESS_FLAG_BIAS_PULL_UP);
    if (ret < 0)
    {
        printf("Failed to set the interrupt\n");
    }
    else
    {
        printf("Passed.\n");
    }

    printf("End --> Completed testing with %s\n", __func__);

    while (1)
    {
        printf("Waiting Interrupt\n");
        sleep(5);
    }

}

void gpio_print_help()
{
    printf("Usage: gpio_test [argument] <chip name/number> <offset>\n");
    printf("\n");
    printf("Argument:\n");
    printf("\t --gpio_output      \t\t execute the gpio output test.\n");
    printf("\t --gpio_output_wait \t\t execute the gpio output wait test.\n");
    printf("\t --gpio_input       \t\t execute the gpio input test.\n");
    printf("\t --gpio_irq_monitor \t\t execute the gpio interrupt monitor test.\n");
}

int main(int argc, char *argv[])
{

    if (argc > 4)
    {
        printf("Only one argument is expected.\n");
        return 0;
    }

    printf("%s\n", argv[1]);
    if (!(strcmp(argv[1], "--gpio_output")) && (argc == 4))
    {
        gpio_output_test(argv[2], atoi(argv[3]));
    }
    else if (!(strcmp(argv[1], "--gpio_output_wait")) && (argc == 4))
    {
        gpio_output_wait_test(argv[2], atoi(argv[3]));
    }
    else if (!(strcmp(argv[1], "--gpio_input")) && (argc == 4))
    {
        gpio_input_test(argv[2], atoi(argv[3]));
    }
    else if (!(strcmp(argv[1], "--gpio_irq_monitor")) && (argc == 4))
    {
        gpio_irq_monitor_test(argv[2], atoi(argv[3]));
    }
    else if (!(strcmp(argv[1], "--help")) && (argc == 2))
    {
        gpio_print_help();
    }
    else
    {
        printf("Invalid argument. Please run with --help to see available arguments.\n");
    }

    return 0;
}