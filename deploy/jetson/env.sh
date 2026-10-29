#!/usr/bin/env bash
# Source this file from the deployed release, including through current/.
robot_release_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
robot_restore_nounset=0
if [[ $- == *u* ]]; then
  robot_restore_nounset=1
  set +u
fi
source /opt/ros/humble/setup.bash
source "${robot_release_dir}/robosub_ws/install/setup.bash"
unset robot_release_dir
if [[ "${robot_restore_nounset}" == 1 ]]; then
  unset robot_restore_nounset
  set -u
else
  unset robot_restore_nounset
fi
