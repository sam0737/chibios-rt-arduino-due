# List of all the STM32F1xx platform files.
PLATFORMSRC += ${CHIBIOS}/os/hal/platforms/SAM3XA/peripheral_config.c \
			   ${CHIBIOS}/os/hal/platforms/SAM3XA/pdc.c \
			   ${CHIBIOS}/os/hal/platforms/SAM3XA/pmc.c \
			   ${CHIBIOS}/os/hal/platforms/SAM3XA/uart_lld.c \
			   ${CHIBIOS}/os/hal/platforms/SAM3XA/serial_lld.c \

# Required include directories
PLATFORMINC += ${CHIBIOS}/os/hal/platforms/SAM3XA \

