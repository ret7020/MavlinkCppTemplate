build:
	mkdir -p bin
	echo Using: ${COMPILER}
	${COMPILER}/riscv64-unknown-linux-musl-g++ main.c -o ./bin/mavlink -Waddress-of-packed-member

deploy:
	scp ./bin/mavlink root@${LICHEERV_IP}:/root/mavlink
