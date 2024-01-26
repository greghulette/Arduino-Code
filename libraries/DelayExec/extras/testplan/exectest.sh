#!/usr/bin/bash

set -euo pipefail

OUTFILE=${1:-}
PORT=${2:-}

if [ -z "${PORT}" ] || [ -n "${3:-}" ]; then
    echo "Usage:"
    echo "  exectest.sh OUTFILE PORT"
    echo "Example:"
    echo "  exectest.sh results.txt /dev/ttyUSB0"
    exit 1
fi

../read_test_result_from_board.sh "${PORT}" > "${OUTFILE}"

