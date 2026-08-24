#!/usr/bin/env bash
# Start FAR-LIO together with a ROS 2 bag player.
#
# Usage: ./run.sh [--rsp] <path-to-rosbag>
#   <path-to-rosbag> is a bag directory (containing metadata.yaml) or an .mcap file.
#   --rsp            also start robot_state_publisher (publishes /tf_static from a URDF)
#
# The bag is mounted into the rosbag container and both services are started.
#
# Optional environment variables:
#   URDF_NAME    URDF file in config/urdf/ used by robot_state_publisher (default KITTI.xml)
set -euo pipefail

usage() {
  echo "Usage: $0 [--rsp] <path-to-rosbag>" >&2
  exit 1
}

with_rsp=0
bag=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --rsp) with_rsp=1 ;;
    -h|--help) usage ;;
    -*) echo "Error: unknown option '$1'" >&2; usage ;;
    *)
      if [ -n "$bag" ]; then
        echo "Error: unexpected argument '$1'" >&2; usage
      fi
      bag="$1"
      ;;
  esac
  shift
done

if [ -z "$bag" ]; then
  usage
fi

if [ ! -e "$bag" ]; then
  echo "Error: '$bag' does not exist" >&2
  exit 1
fi

# Run from the directory containing this script (and docker-compose.yml).
cd "$(dirname "$(realpath "$0")")"

BAG_PATH="$(realpath "$bag")"
BAG_NAME="$(basename "$BAG_PATH")"
export BAG_PATH BAG_NAME

profiles=(--profile far-lio --profile bag)
if [ "$with_rsp" -eq 1 ]; then
  profiles+=(--profile rsp)
fi

exec docker compose "${profiles[@]}" up
