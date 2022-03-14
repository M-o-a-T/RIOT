/*
 * Copyright (C) 2022 Matthias Urlichs
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for
 * more details.
 */

/**
 * @ingroup     cpu_native
 * @ingroup     drivers_periph_gpio
 * @{
 *
 * @file
 * @brief       pseudo-GPIO via Unix socket
 *
 * @author      Matthias Urlichs <matthias@urlichs.de>
 *
 * This driver implements "native" GPIO pins by way of a Unix socket.
 *
 * Pin state is sent and received by the server that listens to the socket,
 * using an n-byte little-endian integer (n=1 for up to 8 pins, n=2 for up
 * to 16 pins, and so on).
 * 
 * Pull-up/down modes are ignored: implementing wired-AND or wired-OR is
 * the server's responsibility.
 *
 * Input and output pin states are not synchronized: you cannot depend on a
 * pin's state to change immediately after setting it.
 */

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "async_read.h"
#include "native_internal.h"
#include "periph/gpio.h"

typedef struct {
    gpio_mode_t mode;
    bool value;
#ifdef MODULE_PERIPH_GPIO_IRQ
    gpio_cb_t cb;
    gpio_flank_t flank;
    void *arg;
    bool enabled;
#endif
} socket_gpio_pin_t;

typedef struct {
    int fd;
    unsigned num_pins;
    socket_gpio_pin_t *pins;
    uint32_t outmask;
} socket_gpio_t;

static socket_gpio_t *ports;
static unsigned num_ports;

#define _port(p)    (p >> GPIO_PORT_SHIFT)
#define _pin(p)     (p & ((1 << GPIO_PORT_SHIFT) - 1))

static void _read_wrapper(int fd, void *arg);

int gpio_socket_setup(const char* socket, uint8_t bits)
{
    struct sockaddr_un address;
    int fd;

    fd = real_socket(PF_UNIX, SOCK_STREAM, 0);
    memset(&address, 0, sizeof(struct sockaddr_un));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, socket, sizeof(address.sun_path)-1);
    if(real_connect(fd, (struct sockaddr *) &address, sizeof(struct sockaddr_un)) != 0) {
        int err = errno;
        real_close(fd);
        return -err;
    }
    real_fcntl(F_SETFL, fd, real_fcntl(F_GETFL, fd, 0) | O_NONBLOCK);

    real_printf("GPIO: connected %d pins via %s\n", bits, socket);

    void *tmp = reallocarray(ports, num_ports + 1, sizeof(*ports));

    if (tmp == NULL) {
        real_close(fd);
        real_printf("GPIO: out of memory\n");
        return -ENOMEM;
    }

    ports = tmp;
    ports[num_ports].fd = fd;
    ports[num_ports].num_pins = bits;
    ports[num_ports].pins = calloc(bits, sizeof(ports[0].pins[0]));
    ports[num_ports].outmask = 0;

    native_async_read_setup();
    native_async_read_add_int_handler(fd, &ports[num_ports], _read_wrapper);

    ++num_ports;
    return num_ports-1;
}

void gpio_socket_teardown(void)
{
    for (unsigned i = 0; i < num_ports; ++i) {
        if (ports[i].pins) {
            real_free(ports[i].pins);
        }
        if (ports[i].fd >= 0) {
            real_close(ports[i].fd);
        }
    }

    if (ports) {
        real_free(ports);
        ports = NULL;
    }

    num_ports = 0;
}

int gpio_init(gpio_t pin, gpio_mode_t mode)
{
    const unsigned p = _pin(pin);
    socket_gpio_t *port;
    socket_gpio_pin_t *ppin;

    if (mode == 0xFF) {
        return -EINVAL;
    }

    if (_port(pin) >= num_ports) {
        return -EINVAL;
    }

    port = &ports[_port(pin)];

    if (p >= port->num_pins) {
        return -EINVAL;
    }
    ppin = &port->pins[p];

    ppin->mode = mode;
    return 0;
}

int gpio_read(gpio_t pin)
{
    const unsigned p = _pin(pin);
    socket_gpio_t *port;
    socket_gpio_pin_t *ppin;

    if (_port(pin) >= num_ports) {
        return -EINVAL;
    }

    port = &ports[_port(pin)];
    if (port->fd < 0) {
        return -EBADF;
    }

    if (p >= port->num_pins) {
        return -EINVAL;
    }

#ifndef MODULE_PERIPH_GPIO_IRQ
    _read_wrapper(port->fd, port);
#endif

    ppin = &port->pins[p];
    return ppin->value;
}

static void _set(gpio_t pin, uint8_t val)
{
    socket_gpio_t *port;
    uint32_t outdata;
    int len;
    int n;

    port = &ports[_port(pin)];
    if(port->fd < 0)
        return;

    if(val)
        port->outmask |= (1<<_pin(pin));
    else
        port->outmask &= ~(1<<_pin(pin));

    outdata = htole32(port->outmask);
    len = (port->num_pins + 7) >> 3;
    n = real_write(port->fd, &outdata, len);

    if(n < len) {
        int err = errno;
        real_close(port->fd);
        port->fd = -1;
        real_printf("GPIO: Write error on port %d/%d: %s\n",
                    _port(pin),_pin(pin), strerror(err));
    }
}

void gpio_set(gpio_t pin)
{
    _set(pin, 1);
}

void gpio_clear(gpio_t pin)
{
    _set(pin, 0);
}

void gpio_toggle(gpio_t pin)
{
    socket_gpio_t *port = &ports[_port(pin)];
    uint32_t m = 1<<_pin(pin);
    _set(pin, !(port->outmask & m));
}

void gpio_write(gpio_t pin, int value)
{
    _set(pin, value);
}

static void _read_wrapper(int fd, void *arg) {
    socket_gpio_t *port = arg;
    socket_gpio_pin_t *ppin;

    if(port->fd < 0) {
        return;
    }
    uint32_t bits = 0;

    int len = (port->num_pins+7)>>3;
    int n = real_read(fd, &bits,len);
    if (n < len) {
        int err = errno;
        real_close(port->fd);
        port->fd = -1;
        real_printf("GPIO: Read error on port %d: %s\n",
                    port-ports, strerror(err));
        return;
    }
    bits = le32toh(bits);

    ppin = &port->pins[0];
    for(unsigned int i = 0; i < port->num_pins; i++) {
#ifdef MODULE_PERIPH_GPIO_IRQ
        if(ppin->value != (bits & 1)) {
            ppin->value = bits & 1;
            if (ppin->cb && ppin->enabled
                && (ppin->flank == GPIO_BOTH
                    || ppin->value == (ppin->flank == GPIO_RISING))) {
                ppin->cb(ppin->arg);
            }
        }
#else
        ppin->value = bits & 1;
#endif
        bits >>= 1;
        ppin++;
    }
#ifdef MODULE_PERIPH_GPIO_IRQ
    native_async_read_continue(fd);
#endif
}

#ifdef MODULE_PERIPH_GPIO_IRQ
#include "async_read.h"

int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                  gpio_cb_t cb, void *arg)
{
    const unsigned p = _pin(pin);
    socket_gpio_t *port;
    socket_gpio_pin_t *ppin;

    if (mode == 0xFF) {
        return -EINVAL;
    }

    if (_port(pin) >= num_ports) {
        return -EINVAL;
    }

    port = &ports[_port(pin)];

    if (p >= port->num_pins) {
        return -EINVAL;
    }

    ppin = &port->pins[p];

    ppin->cb   = cb;
    ppin->arg  = arg;
    ppin->flank = flank;
    ppin->enabled = true;

    return 0;
}

static void _set_irq_enabled(gpio_t pin, bool enabled)
{
    socket_gpio_t *port;
    const unsigned p = _pin(pin);

    if (_port(pin) >= num_ports) {
        return;
    }

    port = &ports[_port(pin)];

    if (p >= port->num_pins) {
        return;
    }

    port->pins[p].enabled = enabled;
}

void gpio_irq_enable(gpio_t pin)
{
    _set_irq_enabled(pin, true);
}

void gpio_irq_disable(gpio_t pin)
{
    _set_irq_enabled(pin, false);
}

#endif /* MODULE_PERIPH_GPIO_IRQ */

/** @} */
