#!/bin/bash
# Run a demo inside the running container.
#   scripts/run_demo.sh joint_control            # interactive: wave / walk / set / list
#   scripts/run_demo.sh dance_basic | dance_beat
#   scripts/run_demo.sh cartesian --left 0.35 0.25 0.05 -90 0 0
cd "$(dirname "$0")/.."
demo="${1:-joint_control}"; shift
docker compose exec -it g1 bash -lc \
  "pkill -f '[j]oint_state_publisher'; source /opt/ros/humble/setup.bash; /usr/bin/python3 -m g1_demos.$demo $*"
