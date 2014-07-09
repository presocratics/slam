#!/usr/bin/env sh
# test.sh
# Martin Miller
# Created: 2014/07/09
# Tests euler2qbw against Matlab outputs.
result=$(../../bin/euler2qbw < euler.txt |paste - Mquat.txt |tr '\t' '\n'|uniq -u|wc -l)
test $result -eq 0 && echo success || echo failure



