"""The reference solution, through the door the grader uses.

The code is `ohm_localization.mcl_node`. This file exists because the simulator loads a controller as a
*file path* — `./lab grade --controller solution/mcl_node.py` imports whatever that path points at, under the
module name `student_node`, and then runs `robot_io.serve()` on it — while `ros2 run ohm_localization
mcl_node` needs an *installed entry point*. Pointing both at one module is what keeps the numbers in
`docs/verification.md` true for whichever door a group used; the alternative, a second copy of the filter
beside the graded one, is how a reference solution drifts away from the thing that was measured.

So: three lines of real content, and the docstring of the module below says what the exercise is and what it
measures. Read `ohm_localization/mcl_node.py`, not this file.
"""
from ohm_localization import mcl_node as _reference

mission = _reference.mission                 # what `serve()` calls once per task
task_options = _reference.task_options       # the parameters the task file asks for
main = _reference.main                       # `python3 solution/mcl_node.py --robot alice` still works

if __name__ == "__main__":
    # Run as a second process against a running simulator (`node controller --controller …`), which is what
    # `ros2 launch ohm_localization mcl.launch.py` does over ROS.
    main()
