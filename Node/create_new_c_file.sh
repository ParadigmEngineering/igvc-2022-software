#!/bin/bash
file_name=$1

touch Core/Inc/$1.h
touch Core/Src/$1.c

echo "// Placeholder" >> Core/Inc/$1.h
echo "// Placeholder" >> Core/Src/$1.c

echo "!Core/Inc/$1.h" >> .gitignore
echo "!Core/Src/$1.c" >> .gitignore

git add -f Core/Inc/$1.h Core/Src/$1.c
