#Shreya Kulsh. Original producer: Andrew Tridgell
#!/bin/bash

# three plane swarm

# assume we start the script from the root directory
ROOTDIR=$PWD
PLANE=$ROOTDIR/build/sitl/bin/arduplane

[ -x "$PLANE" ] || {
    ./waf configure --board sitl
    ./waf plane
}

# setup for multicast
UARTA="tcp:localhost:12345"

PLANE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/plane.parm"

mkdir -p swarm/plane1 swarm/plane2 swarm/plane3
$PLANE --model plane --home -35.3632627,149.1652382,584.090026855469,0 --uartA $UARTA --defaults $PLANE_DEFAULTS

wait
