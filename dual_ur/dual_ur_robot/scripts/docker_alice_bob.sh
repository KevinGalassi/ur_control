#!/bin/bash
docker-compose -p ursim_rightarm_leftarm -f $(rospack find dual_ur_robot)/resources/docker-compose.yml up
