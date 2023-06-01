#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "peripheral_gpio.h"
#include "../i2c/i2c.h"

static char* i2c_device = "/dev/i2c-3";

void peri_gpio_lna_enable_test(void);
void peri_gpio_pa_enable_test(void);
void peri_gpio_cntl1_test(void);
void peri_gpio_cntl2_test(void);
void peri_gpio_cntl3_test(void);
void peri_gpio_cntl4_test(void);
void peri_gpio_cntl5_test(void);
void peri_gpio_cntl6_test(void);
void peri_gpio_mixer1_mode_test(void);
void peri_gpio_mixer2_mode_test(void);
void peri_gpio_switch_v1_1_test(void);
void peri_gpio_switch_v2_1_test(void);
void peri_gpio_switch_v1_2_test(void);
void peri_gpio_switch_v2_2_test(void);

typedef struct {
    char* name;
    void (*test)(void);
} FunctionEntry;

FunctionEntry functionList[] = 
{
    {"peri_gpio_lna_enable",        &peri_gpio_lna_enable_test},
    {"peri_gpio_pa_enable",         &peri_gpio_pa_enable_test},
    
    {"peri_gpio_cntl1",             &peri_gpio_cntl1_test},
    {"peri_gpio_cntl2",             &peri_gpio_cntl2_test},
    {"peri_gpio_cntl3",             &peri_gpio_cntl3_test},
    {"peri_gpio_cntl4",             &peri_gpio_cntl4_test},
    {"peri_gpio_cntl5",             &peri_gpio_cntl5_test},
    {"peri_gpio_cntl6",             &peri_gpio_cntl6_test},

    {"peri_gpio_mixer1_mode",       &peri_gpio_mixer1_mode_test},

    {"peri_gpio_mixer2_mode",       &peri_gpio_mixer2_mode_test},

    {"peri_gpio_switch_v1_1",       &peri_gpio_switch_v1_1_test},

    {"peri_gpio_switch_v2_1",       &peri_gpio_switch_v2_1_test},

    {"peri_gpio_switch_v1_2",       &peri_gpio_switch_v1_2_test},

    {"peri_gpio_switch_v2_2",       &peri_gpio_switch_v2_2_test},
};

void peri_gpio_lna_enable_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_lna_enable(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_pa_enable_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_pa_enable(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_cntl1_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_cntl1(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_cntl2_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_cntl2(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_cntl3_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_cntl3(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_cntl4_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_cntl4(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_cntl5_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_cntl5(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_cntl6_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_cntl6(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_mixer1_mode_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_mixer1_mode(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_mixer2_mode_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_mixer2_mode(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_switch_v1_1_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_switch_v1_1(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_switch_v2_1_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_switch_v2_1(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_switch_v1_2_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_switch_v1_2(&gpio, 1);
    peri_gpio_close(&gpio);
}

void peri_gpio_switch_v2_2_test(void)
{
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, i2c_device, I2C_GPIO_ADDRESS);
    peri_gpio_switch_v2_2(&gpio, 1);
    peri_gpio_close(&gpio);
}

static void peripheral_gpio_print_help()
{
    printf("\n\n");
    printf("USAGE: ./peripheral_gpio_test [argument]\n");
    printf("Example: ./peripheral_gpio_test 10\n");
    printf("\n");
    printf("Argument :\n");
    printf("\n");

    for (int i=0; i < sizeof(functionList) / sizeof(functionList[0]); i++)
    {
        printf("%d\t - %s\n", (i+1), functionList[i].name);
    }
    printf("\n");
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
        peripheral_gpio_print_help();
        return 0;
    }

    int func_num = atoi(argv[1]);
    if (func_num > 0 && func_num < 14)
    {
        functionList[func_num - 1].test();
        return 0;
    }

    printf("Invalid test name. Please type --help to look for available test case.\n");
    return 0;
}