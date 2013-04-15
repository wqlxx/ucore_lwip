CC:=gcc
#CCFLAGS:=-Wall -Iinclude
CCFLAGS:=-Iinclude
BIN:=bin

all: client ftpd print_server

ftpd:server.c lib/utils.c lib/zlog.c lib/vars.c
	echo + ftpd
	${CC} ${CCFLAGS} -o./${BIN}/$@ $^ 

client:client.c lib/utils.c lib/zlog.c lib/vars.c
	echo + client
	${CC} ${CCFLAGS} -o./${BIN}/$@ $^

print_server:test/print_server.c lib/utils.c lib/zlog.c
	echo + print_server
	${CC} ${CCFLAGS} -o./${BIN}/$@ $^

clean:
	rm ./${BIN}/*.o
	rm ./${BIN}/ftp*
	rm ./${BIN}/print_server
