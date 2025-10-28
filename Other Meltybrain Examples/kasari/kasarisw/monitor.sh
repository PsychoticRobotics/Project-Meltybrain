#!/bin/sh
picocom --baud 115200 -r -l -c -e x --imap lfcrlf $1
