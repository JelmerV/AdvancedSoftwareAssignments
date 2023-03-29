# regular c++ compilation and running

tasks.json

``` json
{
"version": "2.0.0",
"tasks": [
{
"type": "cppbuild",
"label": "C/C++: gcc build active file",
"command": "/usr/bin/gcc",
"args": [
"-fdiagnostics-color=always",
"-g",
"${file}",
"-o",
"${fileDirname}/${fileBasenameNoExtension}",
"-lpthread","-lrt"
],
"options": {
"cwd": "${fileDirname}"
},
"problemMatcher": [
"$gcc"
],
"group": "build",
"detail": "compiler: /usr/bin/gcc"
}
]
```

launch.json
```json
{
// Use IntelliSense to learn about possible attributes.
// Hover to view descriptions of existing attributes.
// For more information, visit:
// https://go.microsoft.com/fwlink/?linkid=830387
"version": "0.2.0",
"configurations": [
{
"name": "gcc - Build and debug active file",
"type": "cppdbg",
"request": "launch",
"program": "${fileDirname}/${fileBasenameNoExtension}",
"args": [],
"stopAtEntry": false,
"cwd": "${fileDirname}",
"environment": [],
"externalConsole": false,
"MIMode": "gdb",
"setupCommands": [
{
"description": "Enable pretty-printing for gdb",
"text": "-enable-pretty-printing",
"ignoreFailures": true
}
],
"preLaunchTask": "C/C++: gcc build active file",
"miDebuggerPath": "/usr/bin/gdb"
}
]
}
```

c_cpp_properties.json
```json
{
"configurations": [
{
"name": "Linux",
"includePath": [
"${workspaceFolder}/**"
],
"defines": [],
"compilerPath": "/usr/bin/gcc",
"cStandard": "gnu99",
"cppStandard": "c++14",
"intelliSenseMode": "linux-gcc-arm"
}
],
"version": 4
}
```

## Real time xenomai

uses xeno-config to get the required flags for the gcc compiler command.
`/usr/xenomai/bin/xeno-config --skin=posix --cflags --ldflags`

This is implemeted in the tasks.json

```json
{
"version": "2.0.0",
"tasks": [
{
"type": "cppbuild",
"label": "C/C++: gcc build active file",
"command": "/usr/bin/gcc",
"args": [
"-fdiagnostics-color=always",
"${file}",
"-o",
"${fileDirname}/${fileBasenameNoExtension}",
"$(/usr/xenomai/bin/xeno-config","--skin=posix","--cflags","--ldflags)"
],
"options": {
"cwd": "${fileDirname}"
},
"problemMatcher": [
"$gcc"
],
"group": "build",
"detail": "compiler: /usr/bin/gcc"
}
]
}
```
