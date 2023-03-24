#!/bin/bash
#usage : cat log.log | extract.sh > log.csv

re=".*\[([+-]?[0-9]+[.][0-9]+)\].*\[$1.*\].*setpoint:\s*([+-]?[0-9]+[.][0-9]+),.*error:\s*([+-]?[0-9]+[.][0-9]+), p.*control:\s*([+-]?[0-9]+[.][0-9]+).*"
echo 'time;setpoint;error;control'
while read p; do
#   echo "$p"
  if [[ $p =~ $re ]] ;then
    echo "${BASH_REMATCH[1]};${BASH_REMATCH[2]};${BASH_REMATCH[3]};${BASH_REMATCH[4]}"
  fi
done