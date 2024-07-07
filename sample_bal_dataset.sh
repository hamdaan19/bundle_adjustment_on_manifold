#!/bin/bash
#set -e

mkdir BAL_data
cd BAL_data
wget https://grail.cs.washington.edu/projects/bal/data/ladybug/problem-49-7776-pre.txt.bz2
bzip2 -d problem-49-7776-pre.txt.bz2
cd ..