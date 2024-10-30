#!/usr/bin/env bash


script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "$script_dir"


for stl_file_name in *.stl
do
    [[ -e "$stl_file_name" ]] || break  # handle the case of no *.stl files
    obj_file_name="${stl_file_name/.stl/.obj}"
    meshlabserver -i "$script_dir/$stl_file_name" -o "$script_dir/$obj_file_name" -m vn
    echo "$stl_file_name -> $obj_file_name"
done
