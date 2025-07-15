#!/bin/bash

JETSON_IP='192.168.50.5'
JETSON_USER='imech'
REMOTE_LAUNCH="source ~/ros2_ws/install/setup.bash && ros2 launch spot_bringup demo.launch.py"

# Trap CTRL+C (SIGINT) und beende sauber
cleanup() {
    echo "Abbruch erkannt. Beende SSH..."
    kill $SSH_PID 2>/dev/null
    exit 1
}

trap cleanup SIGINT

# SSH direkt ausführen, aber über stdbuf, damit output nicht gepuffert wird
ssh -t $JETSON_USER@$JETSON_IP "$REMOTE_LAUNCH"
SSH_EXIT=$?

if [ $SSH_EXIT -ne 0 ]; then
    echo "SSH-Verbindung wurde unterbrochen oder beendet."
else
    echo "SSH-Verbindung wurde normal beendet."
fi
