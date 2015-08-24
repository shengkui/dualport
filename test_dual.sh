#!/bin/bash

#The list of serial ports need to be tested.
SERIAL_LIST=(ttyS4 ttyS5 ttyS6 ttyS7)

#if arguments specified, get the serial port list from arguments
if [ $# -gt 0 ];then
    SERIAL_LIST=( $@ )
fi

BAUDRATE=115200
LOOP_COUNT=100

if [ -f "./dualport" ] ;then
    DUALPORT=./dualport
else
    DUALPORT=dualport
fi

term_exit()
{
    echo "Test aborted"
    pkill $DUALPORT
    exit 1
}
trap term_exit TERM INT

#Result of test, 0: PASS, 1: FAIL
result=0

count=${#SERIAL_LIST[@]}
n=0
while [ $n -lt $count ] ;do
    sport1=${SERIAL_LIST[$n]}
    let n=n+1

    if [ $n -ge $count ] ;then
        break;
    fi

    sport2=${SERIAL_LIST[$n]}
    let n=n+1

    echo "Run test on $sport1 and $sport2 ..."

    $DUALPORT /dev/$sport1 /dev/$sport2 -b $BAUDRATE -l $LOOP_COUNT
    if [ $? -ne 0 ];then
        result=1
        echo "ERROR on $sport1 and $sport2"
    fi
done

if [ $result -eq 0 ] ;then
    echo "PASS"
else
    echo "FAIL"
fi

exit $result
