#!/bin/bash

BASEDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ ! -f /usr/local/bin/pan_tilt.sh ]; then
    ln -s ${BASEDIR}/pan_tilt.sh /usr/local/bin/pan_tilt.sh
fi

if [ ! -f /etc/supervisor/conf.d/pan_tilt.conf ]; then
    ln -s ${BASEDIR}/pan_tilt.conf /etc/supervisor/conf.d/pan_tilt.conf
fi

supervisorctl reread
supervisorctl update
