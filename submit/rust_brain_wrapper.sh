#!/bin/bash
# Use relative path to the binary in the same directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
exec "$DIR/rust_brain" "$@"
