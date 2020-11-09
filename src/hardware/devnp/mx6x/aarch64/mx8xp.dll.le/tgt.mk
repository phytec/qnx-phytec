override NAME=devnp
USEFILE=$(PROJECT_ROOT)/$(NAME)-mx8xp.use

override CCOPTS+= -DMX8XP
override define PINFO
PINFO DESCRIPTION=Freescale i.MX8XP ENET driver
endef
