#!/bin/bash
#set -x

rm -f imx-fspi-header
rm -f output.out

while IFS='' read -r line || [[ -n "$line" ]]; do
    v=$(echo "$line" | grep -o '0x.*  ')
    v=${v#0x}
    echo "${v:6:2}${v:4:2}${v:2:2}${v:0:2}" >> output.out
done < imx-fspi-header.src

xxd -r -p -l 512 output.out imx-fspi-header.bin

rm -f output.out
