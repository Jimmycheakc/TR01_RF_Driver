#ifndef __GPIO_TEST_H__
#define __GPIO_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

int interrupt_handler(int event, unsigned int offset, const struct timespec *timestamp, void *userdata);


#ifdef __cplusplus
}
#endif

#endif // __GPIO_TEST_H__