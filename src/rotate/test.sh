#!/usr/bin/env sh
# test.sh
# Martin Miller
# Created: 2014/08/12
# Tests rotate on a sample dataset.
# If diff returns nothing, test is sucessful.
echo "Begin test."
../../bin/rotate 0 0 p < testing/acc > testing/testout
DIFF=$(diff testing/testout testing/acc.rot)
echo "${#DIFF} errors found."
echo "Test complete. Test data saved to testing/testout."

