# List of all the STM32F1xx platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/platforms/SAM3X8E/hal_lld.c \
			  ${CHIBIOS}/os/hal/platforms/SAM3X8E/pal_lld.c \

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/platforms/SAM3X8E \

include $(CHIBIOS)/os/hal/platforms/SAM3XA/platform_base.mk