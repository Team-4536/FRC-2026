from enum import Enum
from subsystems.autoStages import (
    AutoStages,
    FollowTrajectory,
    OperateIntake,
    OperateTurret,
)
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.utils import matchData
from typing import List
from wpilib import SendableChooser, SmartDashboard


class AutoRoutines(Enum):

    DO_NOTHING = "Do Nothing"

    L_BACK_N_SHOOT = "L Back And Shoot"
    M_BACK_N_SHOOT = "M Back And Shoot"
    R_BACK_SHOOT = "R Back And Shoot"

    L_BACK_SHOOT_N_LEAVE = "L Back, Shoot, and Leave"
    M_BACK_SHOOT_N_LEAVE = "M Back, Shoot, and Leave"
    R_BACK_SHOOT_N_LEAVE = "R Back, Shoot, and Leave"

    L_COLLECT_N_SHOOT_X1 = "L Collect and Shoot x1"
    R_COLLECT_N_SHOOT_X1 = "R Collect and Shoot x1"

    L_COLLECT_N_SHOOT_X2 = "L Collect and Shoot x2"
    R_COLLECT_N_SHOOT_X2 = "R Collect and Shoot x2"

    L_COLLECT_N_SHOOT_X3 = "L Collect and Shoot x3"
    R_COLLECT_N_SHOOT_X3 = "R Collect and Shoot x3"

    SYSTEM_CHECK = "System Check"


class AutoSubsystem(Subsystem):
    # Declare Variables
    autoRoutineChooser: SendableChooser = SendableChooser()
    routineFinished: bool = False
    routineKeys: List[str] = list()
    currentPath: int = 0

    def __init__(self):
        super().__init__()

        self.autoRoutineChooser.setDefaultOption(
            AutoRoutines.DO_NOTHING.value,
            AutoRoutines.DO_NOTHING,
        )
        for routine in AutoRoutines:
            self.autoRoutineChooser.addOption(routine.value, routine)

        SmartDashboard.putData("auto routine chooser", self.autoRoutineChooser)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self.routine: dict[str, List[AutoStages]] = routineChooser(
            self.autoRoutineChooser.getSelected(), matchData.isRed()
        )

        self.currentPath = 0
        self.routineFinished = False
        self.routineKeys = list(self.routine.keys())

        SmartDashboard.putStringArray("routineKeys", self.routineKeys)

        if self.routine:
            for path in self.routine[self.routineKeys[self.currentPath]]:
                robotState = path.autoInit(robotState)

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        self.routineFinished = self.currentPath >= len(self.routineKeys)

        if not self.routineFinished:
            for path in self.routine[self.routineKeys[self.currentPath]]:
                robotState = path.run(robotState)
            if self.routine[self.routineKeys[self.currentPath]][0].isDone():
                for path in self.routine[self.routineKeys[self.currentPath]]:
                    robotState = path.end(robotState)
                self.currentPath += 1
                self.routineFinished = self.currentPath >= len(self.routineKeys)
                if not self.routineFinished:
                    for path in self.routine[self.routineKeys[self.currentPath]]:
                        robotState = path.autoInit(robotState)

        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


def routineChooser(
    selectedRoutine: AutoRoutines, isFlipped: bool
) -> dict[str, List[AutoStages]]:
    routine: dict[str, List[AutoStages]] = dict()

    match selectedRoutine:

        case AutoRoutines.DO_NOTHING:
            pass

        case AutoRoutines.SYSTEM_CHECK:
            routine["rev turret"] = [
                OperateTurret(False, 2),
            ]
            routine["shoot turret"] = [
                OperateTurret(True, 2),
            ]
            routine["intake"] = [
                OperateIntake(3),
            ]
            routine["back"] = [
                FollowTrajectory(
                    "M Backward",
                    isFlipped,
                ),
            ]
            routine["for"] = [
                FollowTrajectory(
                    "Forward",
                    isFlipped,
                ),
                OperateTurret(True),
                OperateIntake(),
            ]

        case AutoRoutines.L_BACK_N_SHOOT:
            routine["back"] = [
                FollowTrajectory(
                    "L Backward",
                    isFlipped,
                ),
                # OperateTurret(),
            ]
            routine["unload 1"] = [OperateTurret(True, 4)]

        case AutoRoutines.M_BACK_N_SHOOT:
            routine["back"] = [
                FollowTrajectory(
                    "M Backward",
                    isFlipped,
                ),
                # OperateTurret(),
            ]
            routine["unload 1"] = [OperateTurret(True, 4)]

        case AutoRoutines.R_BACK_SHOOT:
            routine["back"] = [
                FollowTrajectory(
                    "R Backward",
                    isFlipped,
                ),
                # OperateTurret(),
            ]
            routine["unload 1"] = [OperateTurret(True, 4)]

        # case AutoRoutines.L_BACK_SHOOT_N_LEAVE:
        #     routine["back"] = [
        #         FollowTrajectory(
        #             "L Backward",
        #             isFlipped,
        #         ),
        #     ]

        # case AutoRoutines.M_BACK_SHOOT_N_LEAVE:
        #     routine["back"] = [
        #         FollowTrajectory(
        #             "M Backward",
        #             isFlipped,
        #         ),
        #     ]

        # case AutoRoutines.R_BACK_SHOOT_N_LEAVE:
        #     routine["back"] = [
        #         FollowTrajectory(
        #             "R Backward",
        #             isFlipped,
        #         ),
        #     ]

        case AutoRoutines.L_COLLECT_N_SHOOT_X1:
            routine["back"] = [
                FollowTrajectory(
                    "L Backward",
                    isFlipped,
                ),
                OperateTurret(),
            ]
            routine["shoot 1"] = [
                OperateTurret(
                    True,
                    4,
                )
            ]
            routine["L Trench In"] = [
                FollowTrajectory(
                    "L Trench In",
                    isFlipped,
                )
            ]
            routine["L collect"] = [
                FollowTrajectory(
                    "L Collect",
                    isFlipped,
                )
            ]

        case AutoRoutines.R_COLLECT_N_SHOOT_X1:
            routine["back"] = [
                FollowTrajectory(
                    "R Backward",
                    isFlipped,
                ),
                OperateTurret(),
            ]

            routine["shoot 1"] = [
                OperateTurret(
                    True,
                    4,
                )
            ]
            routine["R Trench In"] = [
                FollowTrajectory(
                    "R Trench In",
                    isFlipped,
                )
            ]
            routine["R collect"] = [
                FollowTrajectory(
                    "R Collect",
                    isFlipped,
                )
            ]

        case AutoRoutines.L_COLLECT_N_SHOOT_X2:
            routine["back"] = [
                FollowTrajectory(
                    "L Backward",
                    isFlipped,
                ),
            ]

        case AutoRoutines.R_COLLECT_N_SHOOT_X2:
            routine["back"] = [
                FollowTrajectory(
                    "R Backward",
                    isFlipped,
                ),
            ]

        case AutoRoutines.L_COLLECT_N_SHOOT_X3:
            routine["back"] = [
                FollowTrajectory(
                    "L Backward",
                    isFlipped,
                ),
            ]

        case AutoRoutines.R_COLLECT_N_SHOOT_X3:
            routine["back"] = [
                FollowTrajectory(
                    "R Backward",
                    isFlipped,
                ),
            ]

        case _:
            pass

    return routine
