#!/bin/bash

# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

current_dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
docker_name="ros2_jazzy_ws"
docker_tag="latest"

# modify the dockerfile as required
docker build \
  -t ${docker_name}:${docker_tag} \
  -f ${current_dir}/../dockerfile/nvidia_jazzy.dockerfile .
