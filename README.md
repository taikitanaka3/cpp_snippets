# cpp_snippets

## How to setup

set up repos with `vcs import --force external<test.repos`

## Make ros package

make ros pkg with `make_ros_pkg.sh "pkg_name"`

## Make plugin panel

make ros pkg with `make_plugin_panel.sh "pkg_name"`

## Test autoware pkg

- test

`./dev.sh t $package_name`

- build selected package

`./dev.sh b $package_name`

- build packages up to

`./dev.sh p $package_name`

```.bash
#!/bin/bash
case $1 in
t)
    colcon test --packages-select "$2" --event-handlers console_direct+
    ;;
d)
    sudo rm -r install/"$2" build/"$2"
    ;;
b)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --packages-select "$2"
    ;;
p)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --packages-up-to "$2"
    ;;
*)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install
    ;;
esac
```
