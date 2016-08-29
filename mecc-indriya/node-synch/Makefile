CONTIKI = $(DROPBOX_HOME)/neighbor-discovery/nddev-contiki

ifndef TARGET
TARGET=sky
endif

CONTIKI_SOURCEFILES += nd-dataqueue.c

DEFINES=NETSTACK_MAC=nullmac_driver,NETSTACK_RDC=indriya_nodesynch_driver,CC2420_CONF_AUTOACK=0

#DEFINES+=IN_INDRIYA=1
#DEFINES+=IN_TWIST=1

#ifdef RBS_TSYNH
#DEFINES+=INDRIYA_RBS
#endif #//

clear:
	rm -rf *.sky *.exe symbols.* obj_* *~
	make clean

tbsynch-indriya:
	make clear
	make indriya-nsynch-tb.upload
	cp indriya-nsynch-tb.sky indriya-nsynch4-tb.exe

tbrbs-indriya:
	make clear
	make indriya-nsynch-rbs.upload
	cp indriya-nsynch-rbs.sky indriya-nsynch-rbs.exe

motelist:
	MOTES=$(shell $(MOTELIST) 2>&- | grep USB | \
	cut -f 4 -d \  | \
	perl -ne 'print $$1 . " " if(m-(/dev/\w+)-);')

reload:
	make sky-reset


include $(CONTIKI)/Makefile.include

