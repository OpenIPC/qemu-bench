obj-m += vpss-introspect.o

KDIR ?= /tmp/openipc-linux-4.9
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
