#!/bin/bash
# Script for connection QGC to remote server runnig the SITL simulation. 
# 
# Sources Used: 
# https://gist.github.com/scy/6781836
# https://www.quora.com/How-do-you-forward-UDP-packets-through-an-SSH-tunnel
# 
# 

usage="$(basename "$0") [-h] [-s SERVER_NAME]
 
A script to setup communication tunnel from remote simulation server to local QGC instance.

where:
    -h  show this help text
    -s server_name  the address of simulation server IP or its hostname"

while getopts ':hs:' option; do
  case "$option" in
    h) echo "$usage"
       exit
       ;;
    s) server_name=$OPTARG
       ;;
    :) printf "missing argument for -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       exit 1
       ;;
   \?) printf "illegal option: -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       exit 1
       ;;
  esac
done

## create named pipe on remote machine (delete a named file aready exists)
ssh $server_name rm /tmp/udp2tcp
ssh $server_name mkfifo /tmp/udp2tcp

## create named pipe on local machine (delete a named file aready exists)
rm /tmp/udp2tcp
mkfifo /tmp/tcp2udp


## start translation of UDP packets to TCP packets on local machine side.
echo "startig local connection to QGC."
netcat -lvp 14551 < /tmp/tcp2udp | netcat -u localhost 14550 > /tmp/tcp2udp &

## opens tunnel from remote machine to local machine
echo "opening connection from remote server."
ssh -C -fR 14551:localhost:14551 $server_name sleep 10

# start translation of remote UDP packets to TCP packets trasfered trought tunnel
echo "Connecting the simulator to the tunnel"
ssh $server_name netcat -lvup 14550 < /tmp/udp2tcp | netcat localhost 14551 > /tmp/udp2tcp


