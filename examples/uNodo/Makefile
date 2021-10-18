# name of the application
APPLICATION = uNodo

# If no BOARD is found in the environment, use this default:
BOARD ?= weact-f411ce

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
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
# include and auto-initialize all available sensors
USEMODULE += saul_default

FEATURES_OPTIONAL += periph_rtc

# include and auto-initialize all available sensors
USEMODULE += saul_default
USEMODULE += mcp2515net

include $(RIOTBASE)/Makefile.include
