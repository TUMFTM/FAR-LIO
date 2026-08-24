#!/usr/bin/env bash
# Start FAR-LIO together with a ROS 2 bag player.
#
# Usage: ./run.sh <path-to-rosbag>
#   <path-to-rosbag> is a bag directory (containing metadata.yaml) or an .mcap file.
#
# The bag is mounted into the rosbag container and both services are started.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <path-to-rosbag>" >&2
  exit 1
fi

if [ ! -e "$1" ]; then
  echo "Error: '$1' does not exist" >&2
  exit 1
fi

# Run from the directory containing this script (and docker-compose.yml).
cd "$(dirname "$(realpath "$0")")"

BAG_PATH="$(realpath "$1")"
BAG_NAME="$(basename "$BAG_PATH")"
export BAG_PATH BAG_NAME

exec docker compose --profile far-lio --profile bag up
