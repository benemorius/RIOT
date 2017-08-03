#!/bin/sh

ETHOS_DIR="$(dirname $(readlink -f $0))"
SUDO=
if [ "$EUID" -ne 0 ]; then
    SUDO=sudo
fi

create_tap() {
    ${SUDO} ip tuntap add ${TAP} mode tap user ${USER}
    ${SUDO} sysctl -w net.ipv6.conf.${TAP}.forwarding=1
    ${SUDO} sysctl -w net.ipv6.conf.${TAP}.accept_ra=0
    ${SUDO} ip link set ${TAP} up
    ${SUDO} ip a a fe80::1/64 dev ${TAP}
    ${SUDO} ip a a fd00:dead:beef::1/128 dev lo
    ${SUDO} ip route add ${PREFIX} via fe80::2 dev ${TAP}
}

remove_tap() {
    ${SUDO} ip tuntap del ${TAP} mode tap
}

cleanup() {
    echo "Cleaning up..."
    remove_tap
    ${SUDO} ip a d fd00:dead:beef::1/128 dev lo
    kill ${UHCPD_PID}
    trap "" INT QUIT TERM EXIT
}

start_uhcpd() {
    ${UHCPD} ${TAP} ${PREFIX} > /dev/null &
    UHCPD_PID=$!
}

PORT=$1
TAP=$2
PREFIX=$3
BAUDRATE=115200
UHCPD="$(readlink -f "${ETHOS_DIR}/../uhcpd/bin")/uhcpd"

[ -z "${PORT}" -o -z "${TAP}" -o -z "${PREFIX}" ] && {
    echo "usage: $0 <serial-port> <tap-device> <prefix> [baudrate]"
    exit 1
}

[ ! -z $4 ] && {
    BAUDRATE=$4
}

trap "cleanup" INT QUIT TERM EXIT


create_tap && start_uhcpd && "${ETHOS_DIR}/ethos" ${TAP} ${PORT} ${BAUDRATE}
