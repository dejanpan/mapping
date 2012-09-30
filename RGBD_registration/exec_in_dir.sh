#!/bin/bash

if [ "$2" == "" ]; then
    echo "Usage: $0 <dir> <cmd> [params*]"
    exit 1
fi

mkdir -p $1
cd $1
exec ${@:2}

