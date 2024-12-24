# Path Calculator
Web application that calculates a path to visit multiple coordinates for the Dragon City minigame Fog Island

## GitHub Page
The web application is deployed at https://cornsnek.github.io/path-calculator/

## About
I was interested in trying to find out how to calculate a path to multiple coordinates of items of the Fog Island minigame.
I built this app to visualize the paths and how the pathfinder calculates the direction and the cost of each node visited.

## Zig Build
This project currently uses Zig 0.13.0 to build the project.

In order to build the website and/or the wasm binary: `zig build wasm -Doptimize=...`

Python 3 is also used to build the server to build and test the website: `zig build server`

## TODO
Create a pathfinder algorithm for the lowest coin cost.

For the Fog Island minigame, there is a mechanic where you can revisit a node or coordinate for a fixed cost of 5 coins.
This makes finding algorithms to calculate the lowest coin cost trickier because part of a previous path is also considered to be the shortest to a targeted coordinate. Fog Island isn't necessarily a Travelling Salesperson Problem (TSP) due to this mechanic.

The algorithm for the pathfinder currently calculates the lowest coin cost for each individual path, but not all the paths in whole.