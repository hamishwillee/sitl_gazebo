#!/bin/bash
# Script for connection QGC to remote server runnig the SITL simulation. 
# 
# Sources Used: 
# https://gist.github.com/scy/6781836
# https://www.quora.com/How-do-you-forward-UDP-packets-through-an-SSH-tunnel
#

## create named pipe on remote machine (delete a named file aready exists)
#ssh $1 [ -e /tmp/udp2tcp ] && rm /tmp/udp2tcp
#ssh $1 mkfifo /tmp/udp2tcp

## create named pipe on local machine (delete a named file aready exists)
#[ -e /tmp/udp2tcp ] && rm /tmp/udp2tcp
#mkfifo /tmp/udp2tcp


## start translation of UDP packets to TCP packets on local machine side.
echo "startig local connection to QGC."
netcat -lvp 14551 < /tmp/tcp2udp | netcat -u localhost 14550 > /tmp/tcp2udp &

## opens tunnel from remote machine to local machine
echo "opening connection from remote server."
ssh -C -fR 14551:localhost:14551 $1 sleep 10

# start translation of remote UDP packets to TCP packets trasfered trought tunnel
echo "Connecting the simulator to the tunnel"
ssh $1 netcat -lvup 14550 < /tmp/udp2tcp | netcat localhost 14551 > /tmp/udp2tcp


