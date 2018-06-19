from actionlib import SimpleActionClient

from flir_pantilt_d46.msg import PtuGotoAction, PtuGotoGoal

from math import pi


class PTUController:
    """
    PTUController is a very simple action server client for the PTU.
    """

    def __init__(self, ptu_topic):
        """
        Initializes the controller

        Parameters
        ----------
        ptu_topic: str
            The topic of the ptu action server.
        """

        self._action_client = SimpleActionClient(ptu_topic, PtuGotoAction)

        self._action_client.wait_for_server()

    def goto(self, pan, vel=0.1):
        """
        Moves the PTU to a defined value of pan.

        Parameters
        ----------
        pan: float
            The target value of pan in radians.
        vel: float
            The angular velocity of the pan. Defaults to 0.1 rads per secounds.
        """

        goal = PtuGotoGoal(
            pan=(pan / pi * 180),
            pan_vel=(vel / pi * 180),
            tilt=0,
            tilt_vel=1)

        self._action_client.send_goal_and_wait(goal)
