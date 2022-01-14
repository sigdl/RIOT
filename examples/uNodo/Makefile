# name of the application
APPLICATION = uNodo

# If no BOARD is found in the environment, use this default:
#BOARD ?= weact-f411ce
BOARD ?= stm32f4discovery

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Uncomment this to enable scheduler statistics for ps:
#USEMODULE += schedstatistics

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

# Modules to include:
#USEMODULE += mcp2515net
USEMODULE += periph_can_netdev
USEMODULE += gnrc_can
#USEMODULE += periph_eth
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
#USEMODULE += ztimer_usec
USEMODULE += can_netdev_utils

# Include and auto-initialize all available sensors
USEMODULE += saul_default

# Use modules for networking
# gnrc is a meta module including all required, basic gnrc networking modules
USEMODULE += gnrc

# use the default network interface for the board
#USEMODULE += netdev_default

# Use CAN module
USEMODULE += gnrc_netif_can

# automatically initialize the network interface
USEMODULE += auto_init_gnrc_netif

# shell command to send L2 packets with a simple string
USEMODULE += gnrc_txtsnd
  
# the application dumps received packets to stdout
USEMODULE += gnrc_pktdump

# Use PM
USEMODULE += pm_layered

FEATURES_OPTIONAL += periph_rtc

# This must go at the end!!
include $(RIOTBASE)/Makefile.include
