#!/usr/bin/env bash

remote_link="https://github.com/MAPF-Competition/Start-Kit.git"
git remote add upstream $remote_link
git fetch upstream --no-tags main

git restore --staged --worktree --source=upstream/main utils/upgrade_file_list.txt

files=""
while read -r line;
do
   files="$files $line" ;
done < utils/upgrade_file_list.txt

git restore --staged --worktree --source=upstream/main $files

git commit -m "updated to latest startkit"
git push origin