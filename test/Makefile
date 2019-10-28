SHELL := /bin/bash

test: test_load test_rx test_tx_rx
	$(info tests passed)

test_load:
	$(info testing dfu load of program)
	cp ../build/motor-tmp.bin .
	cp ../build/motor-tmp_param.bin .
	../load_program.sh
	rm motor-tmp*.bin
	sleep 1

test_rx:
	$(info testing usb receive)
	head -c100 /dev/skel0

test_tx_rx:
	$(info testing usb send and receive)
	perl -e 'print pack("I", $$ARGV[0])' 12345678 > /dev/skel0
	rec=($$(od -An -i -N20 -w20 /dev/skel0)); \
	(( $${rec[1]} == 12345678 ))

