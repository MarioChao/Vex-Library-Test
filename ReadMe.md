# Vex Library Test

Test out C++ program locally **without** connecting to a V5 Brain!


## Libraries

The repository contains part of [PAS1](https://mariochao.github.io/vex-pas1/)'s robot program that runs purely in C++11, compatible with [VEXcode Api](https://api.vex.com/).

The included libraries are:
- [Aespa Library](./include/Aespa-Lib/): utility library
- [Pas1 Library](./include/Pas1-Lib/) (Path Planning): utility library for coding **autonomous**


## Set up

To compile the program, use the `make` command in the terminal.
This creates the `all` executable file that you can run locally.

The test files are saved to the directories `/dev-files/paths/`, `/dev-files/polygons/`, etc.<br>
Make sure these folders exist in order for the test data to appear!


## Test Graphs

Check [graphs.ipynb](/graphs.ipynb) to see a visualized view of test results!
