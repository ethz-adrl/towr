#!/bin/bash

# set environmental variable to the parent directory of this script
export XPP_ROOT=$(cd `dirname "${BASH_SOURCE[0]}"` && cd .. && pwd)

alias ua='cd $XPP_ROOT'
alias bina='cd $XPP_ROOT/bin'
alias cma='cd $XPP_ROOT/build'
