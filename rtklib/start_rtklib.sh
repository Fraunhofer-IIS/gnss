#!/bin/sh
conf="${1:-"enu_rtk.conf"}"
rtklib_dir=$(dirname $0)
$rtklib_dir/bin/rtkrcv -s -o $rtklib_dir/$conf -p 3000
