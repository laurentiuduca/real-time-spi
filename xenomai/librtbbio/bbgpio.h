#ifndef BBGPIO_H_
#define BBGPIO_H_

#include <rtdm/rtdm.h>

//! code indicating GPIO input direction
#define BBGPIO_DIR_IN             1
//! code indicating GPIO output direction
#define BBGPIO_DIR_OUT            2
//! code indicating GPIO IRQ input direction
#define BBGPIO_DIR_IRQ            3

//! code indicating GPIO IRQ rising edge
#define BBGPIO_IOCTL_EDGE_RISING        1
//! code indicating GPIO IRQ falling edge
#define BBGPIO_IOCTL_EDGE_FALLING       2
//! code indicating GPIO IRQ both edges
#define BBGPIO_IOCTL_EDGE_BOTH          3

//! IOCTL code for getting GPIO direction
#define BBGPIO_IOCTL_DIR_GET \
    _IOR(RTDM_CLASS_EXPERIMENTAL, 0x01, int)
//! IOCTL code for getting GPIO IRQ edge
#define BBGPIO_IOCTL_EDGE_GET \
    _IOR(RTDM_CLASS_EXPERIMENTAL, 0x02, int)
//! IOCTL code for setting GPIO IRQ edge
#define BBGPIO_IOCTL_EDGE_SET \
    _IOW(RTDM_CLASS_EXPERIMENTAL, 0x03, int)
//! IOCTL code for getting GPIO IRQ timeout in nanoseconds
#define BBGPIO_IOCTL_TIMEOUT_GET \
    _IOR(RTDM_CLASS_EXPERIMENTAL, 0x04, int)
//! IOCTL code for setting GPIO IRQ timeout in nanoseconds
#define BBGPIO_IOCTL_TIMEOUT_SET \
    _IOW(RTDM_CLASS_EXPERIMENTAL, 0x05, int)

#endif /* BBGPIO_H_ */
