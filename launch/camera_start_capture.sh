#!/bin/bash
rosservice call --wait /camera/start_capture &
exec "$@"
