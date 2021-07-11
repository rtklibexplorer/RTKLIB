#! /bin/bash
echo -en "Message [Checksum] =" $* "["

# Calculate the Hexadecimal message checksum using bc
MESSAGE=$*
#"06 0C 04 00 17 24 04 00"
SUM=0
FLETCHER=0
j=0

for i in $MESSAGE
do
 j=$(echo "ibase=16;$i" | bc)
# printf "%x " "$j"

 SUM=$(echo "$SUM + $j" | bc)
 SUM=$(echo "$SUM%256" | bc)

 FLETCHER=$(echo "$FLETCHER + $SUM" | bc)
 FLETCHER=$(echo "$FLETCHER%256" | bc)
done
printf "%x " "$SUM"
printf "%x]\n" "$FLETCHER"
