#!/usr/bin/bash

set -euo pipefail

cd main

for d in [0-9][0-9]; do
    rm -f "${d}"/tmpout*.txt
done

cd ../main2

for d in [0-9][0-9]; do
    rm -f "${d}"/tmpout*.txt
done

