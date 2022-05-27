#!/bin/bash
file_name=$1

touch Core/Inc/$file_name.h
touch Core/Src/$file_name.c

echo "// Placeholder" >> Core/Inc/$file_name.h
echo "// Placeholder" >> Core/Src/$file_name.c

echo "!Core/Inc/$file_name.h" >> .gitignore
echo "!Core/Src/$file_name.c" >> .gitignore

git add -f Core/Inc/$file_name.h Core/Src/$file_name.c
