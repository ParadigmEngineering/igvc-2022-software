# igvc-comp1-software
Intelligent Ground Vehicle Competition (Spring 2022) software for the Paradigm Engineering Student Group

# General Cloning Advice
Be sure to recurse those submodules. 

If before cloning
```
 git clone --recursive git@github.com:ParadigmBoring/igvc-comp1-software.git
```

If after cloning
```
git submodule update --init --recursive
```

# ROS Setup
ROS packages are handled a little differently in this repo than most others.

## TLDR
Prerequisites:
- Linux
- ROS install + ROS build tools

From repo root:
```
./rospack_setup.sh
pmake
```

## More Info
ROS best practice states that ROS packages should be developed in a workspace. Modern 
ROS distros use the catkin build tool, and catkin workspaces. The catkin workspace consists
of three "spaces", which are just folders:
- `build` (build-space)
- `devel` (develop-space)
- `src`   (source-space)

Packages are stored in the `src` space. This makes it very easy to build all packages 
and manage cross dependencies (the primary advantage of keeping them in the workspace).
However, there are a lot of transient files generated in devel and build as a result.

To avoid tracking these in git, the `src` folder of the catkin workspace is symlinked
to the `ROS` folder of this repo. This is setup by the script `rospack_install.sh`. This allows
us to track only the packages, and also easily configure a catkin workspace. This script
also sets up the `pmake` alias which calls catkin_make in the created workspace. The script
also adds some useful lines to `bashrc`, like sourcing the workspace setup script.  

The script creates a catkin workspace at `~/catkin_ws`, and establishes the symlink. Keep in
mind:
- Only valid on linux
- Script must be executed from the repo root
- Must havbe ROS installed (a new enough distro that catkin is used - Noetic ideally)

After executing the script, the following should build all packages in the repository
after which they can be spun up using `rosrun` or `roslaunch`:
```
pmake
```

**Note: I have not seen this approach used before. There may be some pitfalls associated
with this that we will absolutely swan dive into. Contingency: track the catkin workspace.**
