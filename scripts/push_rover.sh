#!/usr/bin/env bash

set -e

if [ -z "$1" ]; then
    echo "Usage:"
    echo "./scripts/push_rover.sh \"Your commit message\""
    exit 1
fi

cd "$HOME/rover_ws"

git status
git add .
git commit -m "$1"
git push

echo "YAMIK Rover backup pushed to GitHub successfully."
