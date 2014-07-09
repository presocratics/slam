#!/usr/bin/env sh
# test.sh
# Martin Miller
# Created: 2014/07/09
# Compare output of img2body with matlab generated output.
../../bin/img2body < pixels.txt|cut -d, -f2,3,4,5|paste - Matout.txt|tr '\t' \
'\n'|sed '/,,,/d'|uniq -u


