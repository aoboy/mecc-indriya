CONTIKI = $(DROPBOX_HOME)/neighbor-discovery/nddev-contiki

ifndef TARGET
TARGET=sky
endif


CONTIKI_SOURCEFILES += nd-dataqueue.c indriya-auto-medal.c
#CONTIKI_SOURCEFILES += nd-dataqueue.c indriya-medal.c

#DEFINES=NETSTACK_MAC=nullmac_driver,NETSTACK_RDC=indriya_automedal_driver,CC2420_CONF_AUTOACK=0
DEFINES=NETSTACK_MAC=nullmac_driver,NETSTACK_RDC=indriya_medal_driver,CC2420_CONF_AUTOACK=0
DEFINES+=CONF_CHANNEL_POOL_SIZE=3,CONF_NETWORK_CLIQUE_SIZE=2
DEFINES+=HOPCOUNT_FILTER_NDISC=15

#DEFINES+=IN_INDRIYA=1
#DEFINES+=IN_TWIST=1


clear:
	rm -rf *.sky *.exe symbols.* obj_* *~
	make clean

indriya-medal:
	make clear
	make indriya-example-medal.upload
	cp indriya-example-medal.sky indriya-example-medal-autoNew2.exe

synchro:
	make clear
	make node-synch.upload
#	make nsynch-rbs.upload

tbsynch-indriya:
	make clear
	make indriya-nsynch-tb.upload
	cp indriya-nsynch-tb.sky tb-bin/indriya-nsynch-tb.exe

tbrbs-indriya:
	make clear
	make indriya-nsynch-rbs.upload
	cp indriya-nsynch-rbs.sky tb-bin/indriya-nsynch-rbs.exe

motelist:
	MOTES=$(shell $(MOTELIST) 2>&- | grep USB | \
	cut -f 4 -d \  | \
	perl -ne 'print $$1 . " " if(m-(/dev/\w+)-);')

reload:
	make sky-reset


include $(CONTIKI)/Makefile.include

