#! /bin/bash

if ! [ -x "$(command -v clang)" ]; then
  echo 'Error: clang is not installed, please install clang or add clang on the path.' >&2
  exit 1
fi

perf record -m 1G,1G -e intel_pt// -- clang lu.c -pthread -lm
rm -rf a.out
# If you want all instructions make full_trace true
full_trace=false
if [ "$full_trace" = true ] ; then
  perf script --itrace=i0ns --ns -F ip,insnlen,insn|awk '{$2="";$4="";print $0}'|gzip -9 > clang.gz
else
  # takes five minutes to run in my machine
  perf script --itrace=i0ns --ns -F ip,insnlen,insn|awk '{$2="";$4="";print $0}'|head -100000000|gzip -9 > clang.gz
fi
rm -rf perf.data