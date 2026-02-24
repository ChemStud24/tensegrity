#!/bin/bash

PYTHON_PATH="$(which python)"
echo "$PYTHON_PATH"

# how to use sed for line replacement: https://stackoverflow.com/a/13438118
# why using '@' instead of '/': https://stackoverflow.com/a/9366940
sed -i "1s@.*@#\!$PYTHON_PATH@" tracking_service.py
sed -i "1s@.*@#\!$PYTHON_PATH@" tracking_client.py
