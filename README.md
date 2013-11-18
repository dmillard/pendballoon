pendballoon simulation
----------------------

## Required libraries:

 - SFML (www.sfml-dev.org)
 - Chipmunk (www.chipmunk-physics.net)

## Download/Compilation/Usage:

```bash
# acquire files
git clone https://github.com/dmillard/pendballoon.git`
# change to file directory
cd pendballoon
# make and enter build environment
mkdir build && cd build
# generate build files
cmake ..
# build project
make
# run simulation
./pendballoon
```

## Todo:
 - Simulate a pendulum with balloon attached
 - Figure out why Chipmunk is modelling energy dissipation
