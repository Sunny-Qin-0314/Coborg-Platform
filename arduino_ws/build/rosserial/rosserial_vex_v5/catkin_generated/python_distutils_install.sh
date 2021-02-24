#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hwadi/Coborg-Platform/arduino_ws/src/rosserial/rosserial_vex_v5"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hwadi/Coborg-Platform/arduino_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hwadi/Coborg-Platform/arduino_ws/install/lib/python2.7/dist-packages:/home/hwadi/Coborg-Platform/arduino_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hwadi/Coborg-Platform/arduino_ws/build" \
    "/usr/bin/python2" \
    "/home/hwadi/Coborg-Platform/arduino_ws/src/rosserial/rosserial_vex_v5/setup.py" \
     \
    build --build-base "/home/hwadi/Coborg-Platform/arduino_ws/build/rosserial/rosserial_vex_v5" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/hwadi/Coborg-Platform/arduino_ws/install" --install-scripts="/home/hwadi/Coborg-Platform/arduino_ws/install/bin"
