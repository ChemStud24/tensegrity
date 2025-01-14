README

This will be the repo where we can open-source our ROS package for controlling the tensegrity.  We should leave the tracking service in its own repo.  I included some "legacy" files so I can still use the old robot (Bluetooth) for testing.  These will go away when I am ready to fully switch over.  The file names have been simplified (e.g. Classes_run_tensegrity.py -> run_tensegrity.py).

Patrick, if you can put your A star planning code into planner.py, I can test it on the old robot and, when I get mine working, the new one too.

I'm working on merging legacy_run_tensegrity_Astar with run_tensegrity_Astar so you can use it for your platform, but I haven't finished yet.  Testing everything is also WIP.