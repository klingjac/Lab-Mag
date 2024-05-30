#!/bin/bash

# Find all processes running Python and terminate them
echo "Terminating all Python processes..."

# Use pgrep to find all python processes and kill to terminate them
# SIGTERM is used to gracefully stop the process
pgrep python | xargs kill

echo "Sent termination signals to all Python processes."

# Optional: Uncomment the following lines to force kill any process that didn't terminate
# sleep 5  # Wait for 5 seconds to allow graceful shutdown
# echo "Force killing any remaining Python processes..."
# pgrep python | xargs kill -9  # SIGKILL to force termination
