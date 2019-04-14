#!/usr/bin/env bash

xargs -a apt_requirements.txt apt-get install -y
pip install -r pip_requirements.txt
