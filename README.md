# eklavya-2015

## The Minimal branch

### Goals of this branch:

1. Refactor code using [SOLID](https://en.wikipedia.org/wiki/SOLID_%28object-oriented_design%29) principles.
2. Keep the repo lean and clean.
4. Make code easier to debug and reuse.

### Things to do:

- [ ] Delete useless stuff.
- [ ] Reorganize code in appropriate folders
- [ ] Design an architecture

## Improvement stats:

|       Parameter  |    igvc_nav     |         minimal    |
|------------------|:---------------:|:------------------:|
| Lines of code:   |    640948       |         489424     |


## Proposed Strucuture

The key idea is that each logical unit (folder/class) can be easily replaced.

  * Environment-
      * Perception-
          * Real-life Perception
          * Simulation
          * Mapping
  * Robot-
      * Sensing-
          * Sensor drivers
      * Controls
      * Localization
      * Simulation
  * Planning-  (This needs to be discussed further)
      * Planner
      * Waypoint_planner
  * Utils-
      * Utility code like teleop, launch files, scripts
