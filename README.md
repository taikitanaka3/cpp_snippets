# cpp_snippets

## How to setup

set up repos with `vcs import --force external<test.repos`

## Make ros package

make ros pkg with `make_ros_pkg.sh "pkg_name"`

## Test autoware pkg

- test

`./dev.sh test`

- build

`./dev.sh`

```.bash
#!/bin/bash
PKG=interp

#PKG=path_smoother
case $1 in
test)
    colcon test --packages-select "$PKG" --event-handlers console_direct+
    ;;
build_all)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --continue-on-error
    ;;
delete)
    sudo rm -r install/"$PKG" build/"$PKG"
    ;;
debug)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install --packages-select "$PKG"
    ;;
*)
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --packages-select "$PKG"
    ;;
esac
```
