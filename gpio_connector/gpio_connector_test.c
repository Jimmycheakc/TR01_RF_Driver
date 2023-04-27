#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gpio_connector.h"

void gpio_connector_rf_1v2_en_test()
{
    printf("Start --> %s\n", __func__);
    int ret;

    printf("1. %s: enable gpio connector rf_1v2_en.\n", __func__);
    ret = gpio_rf_1v2_en_enabled();
    if (ret < 0)
    {
        printf("Failed to enable rf_1v2_en.\n");
    }
    else
    {
        printf("Passed.\n");
    }

    printf("1. %s: disable gpio connector rf_1v2_en.\n", __func__);
    ret = gpio_rf_1v2_en_disabled();
    if (ret < 0)
    {
        printf("Failed to disable rf_1v2_en.\n");
    }
    else
    {
        printf("Passed.\n");
    }

    printf("End --> Completed testing with %s\n", __func__);
}

void gpio_connector_rst_rf_b_test()
{
    printf("Start --> %s\n", __func__);
    int ret;

    printf("1. %s: enable gpio connector rst_rf_b.\n", __func__);
    ret = gpio_rst_rf_b_enabled();
    if (ret < 0)
    {
        printf("Failed to enable rst_rf_b.\n");
    }
    else
    {
        printf("Passed.\n");
    }

    printf("1. %s: disable gpio connector rst_rf_b.\n", __func__);
    ret = gpio_rst_rf_b_disabled();
    if (ret < 0)
    {
        printf("Failed to disable rst_rf_b.\n");
    }
    else
    {
        printf("Passed.\n");
    }

    printf("End --> Completed testing with %s\n", __func__);
}

void gpio_connector_rbe_test()
{
    printf("Start --> %s\n", __func__);
    int ret;

    printf("1. %s: enable gpio connector rbe.\n", __func__);
    ret = gpio_rbe_enabled();
    if (ret < 0)
    {
        printf("Failed to enable rbe.\n");
    }
    else
    {
        printf("Passed.\n");
    }

    printf("1. %s: disable gpio connector rbe.\n", __func__);
    ret = gpio_rbe_disabled();
    if (ret < 0)
    {
        printf("Failed to disable rbe.\n");
    }
    else
    {
        printf("Passed.\n");
    }

    printf("End --> Completed testing with %s\n", __func__);
}

void gpio_connector_print_help(void)
{
    printf("Usage: ./gpio_connector_test [argument]\n");
    printf("\n");
    printf("Argument:\n");
    printf("\t --rf_1v2_en      \t\t execute the gpio connector rf_1v2_en test.\n");
    printf("\t --rst_rf_b       \t\t execute the gpio connector rst_rf_b test.\n");
    printf("\t --rbe            \t\t execute the gpio connector rbe test.\n");
}

int main (int argc, char *argv[])
{
    if (argc > 2)
    {
        printf("Only one argument is expected.\n");
        return 0;
    }

    if (!(strcmp(argv[1], "--rf_1v2_en")) && (argc == 2))
    {
        gpio_connector_rf_1v2_en_test();
    }
    else if (!(strcmp(argv[1], "--rst_rf_b")) && (argc == 2))
    {
        gpio_connector_rst_rf_b_test();
    }
    else if (!(strcmp(argv[1], "--rbe")) && (argc == 2))
    {
        gpio_connector_rbe_test();
    }
    else if (!(strcmp(argv[1], "--help")) && (argc == 2))
    {
        gpio_connector_print_help();
    }
    else
    {
        printf("Invalid argument. Please run with --help to see available arguments.\n");
    }

    return 0;
}