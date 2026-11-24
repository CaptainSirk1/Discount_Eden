#!/bin/bash
# usage: sweep.sh tag "args..." ["args..."] ...   runs pairs in parallel, prints t=0/2000/4000 summary
tag=$1; shift
H=/home/claude/work/harness/bin/Harness; R=/home/claude/work/results
i=0; pids=()
for a in "$@"; do
  i=$((i+1)); n="${tag}_$i"
  ( $H $a out=$R/$n.csv > $R/$n.txt 2>&1 ) & pids+=($!)
  if (( ${#pids[@]} >= 2 )); then wait ${pids[0]}; pids=("${pids[@]:1}"); fi
done
wait
i=0
for a in "$@"; do
  i=$((i+1)); n="${tag}_$i"
  echo "== $n: $a"
  grep -E "^ +[0-9]" $R/$n.txt | awk 'NR==1 || NR%4==1' | cut -c1-12,24-53,96-120,124-147,160-176 | head -12
  grep -E "^ +[0-9]" $R/$n.txt | tail -1 | sed 's/.*| /   /'
done
