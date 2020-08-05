#!/usr/bin/env bash
#
# Copyright 2017 - 2018 Ternaris
# SPDX-License-Identifier: Apache 2.0

for x in /opt/*; do
    if [[ -e "$x/.env.sh" ]]; then
	     source "$x/.env.sh"
    fi
done

cd || exit 1
