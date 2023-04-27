#include <stdio.h>
#include <stdint.h>
#include "peripheral_gpio.h"

typedef struct {
    char* name;
    void (*test)(void);
} FunctionEntry;

FunctionEntry functionList{
    
    {peri_gpio_lna_enable,        &peri_gpio_lna_enable_test},
    {peri_gpio_pa_enable,         &peri_gpio_pa_enable_test},
    
    {peri_gpio_cntl1,             &peri_gpio_cntl1_test},
    {peri_gpio_cntl2,             &peri_gpio_cntl2_test},
    {peri_gpio_cntl3,             &peri_gpio_cntl3_test},
    {peri_gpio_cntl4,             &peri_gpio_cntl4_test},
    {peri_gpio_cntl5,             &peri_gpio_cntl5_test},
    {peri_gpio_cntl6,             &peri_gpio_cntl6_test},

    {peri_gpio_mixer1_reset,      &peri_gpio_mixer1_reset_test},
    {peri_gpio_mixer1_enable,     &peri_gpio_mixer1_enable_test},
    {peri_gpio_mixer1_enable_bl,  &peri_gpio_mixer1_enable_bl_test},
    {peri_gpio_mixer1_mode,       &peri_gpio_mixer1_mode_test},

    {peri_gpio_mixer2_reset,      &peri_gpio_mixer2_reset_test},
    {peri_gpio_mixer2_enable,     &peri_gpio_mixer2_enable_test},
    {peri_gpio_mixer2_enable_bl,  &peri_gpio_mixer2_enable_bl_test},
    {peri_gpio_mixer2_mode,       &peri_gpio_mixer2_mode_test},
};

void peripheral_gpio_print_help()
{
    printf("Usage: ./peripheral_gpio_test [argument]\n");
    printf("\n");
    printf("Argument:\n");
    printf("\t --rf_1v2_en      \t\t execute the gpio connector rf_1v2_en test.\n");
    printf("\t --rst_rf_b       \t\t execute the gpio connector rst_rf_b test.\n");
    printf("\t --rbe            \t\t execute the gpio connector rbe test.\n");
}

int main(int argc, char *argv[])
{

    if (argc > 2)
    {
        printf("Only one argument is expected.\n");
        return 0;
    }

    if (!(strcmp(argv[1], "--help")))
    {
        print_help();
        return 0;
    }

    int func_num = atoi(argv[1]);
    if (func_num > 0 && func_num < 21)
    {
        functionList[func_num - 1].test();
        return 0;
    }

    printf("Invalid test name. Please type --help to look for available test case.\n");
    return 0;
}