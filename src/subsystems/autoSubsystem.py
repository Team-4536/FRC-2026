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
    A_EMPTY_AND_GET_BALLS_AND_SHOOT_FROM_MID = "Empty and get balls and shoot from mid"
    A_EMPTY_AND_GET_BALLS_AND_SHOOT_FROM_RIGHT = (
        "Empty and get balls and shoot from right"
    )
    A_EMPTY_AND_GET_BALLS_AND_SHOOT_FROM_LEFT = (
        "Empty and get balls and shoot from left"
    )
    DO_NOTHING = "Do Nothing"
    JUST_SHOOT = "Just Shoot"
    TEST_INTAKE = "Test Intake"
    DRIVE_FORWARD_BACK_TEST = "Drive Forward Back Test"
    WONKY = "Wonky"
    GET_BALLS_RIGHT_FROM_HUB = "Get Balls Right From Hub"
    GET_BALLS_LEFT_FROM_HUB = "Get Balls Left From Hub"
    GET_BALLS_RIGHT_FROM_START = "Get Balls Right From START"
    GET_BALLS_LEFT_FROM_START = "Get Balls Left From START"
    MESS_WITH_MIDDLE_FROM_RIGHT = "Mess With Middle From Right"
    DROP_INTAKE = "Drop Intake"
    FORWARD = "Forward"
    FORWARD_AND_INTAKE = "Forward And Intake"
    FORWARD_AND_SHOOT = "Forward And Shoot"
    L_TEST = "L Test"


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
            self.autoRoutineChooser.getSelected(), matchData.isBlue()
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

        case AutoRoutines.A_EMPTY_AND_GET_BALLS_AND_SHOOT_FROM_MID:
            routine["Back and rev"] = [
                FollowTrajectory(
                    "Backward",
                    isFlipped,
                ),
                OperateTurret(),
            ]
            routine["shoot"] = [
                OperateTurret(
                    True,
                    5,
                )
            ]
            routine["under right trench from start middle"] = [
                FollowTrajectory(
                    "under right trench from start middle",
                    isFlipped,
                )
            ]
            routine["right to balls"] = [
                FollowTrajectory(
                    "right to balls",
                    isFlipped,
                )
            ]
            routine["under left trench to hub"] = [
                FollowTrajectory(
                    "under left trench to hub",
                    isFlipped,
                )
            ]
            routine["shoot"] = [
                OperateTurret(
                    True,
                    8,
                )
            ]

        case AutoRoutines.A_EMPTY_AND_GET_BALLS_AND_SHOOT_FROM_RIGHT:
            routine["shoot"] = [
                OperateTurret(
                    True,
                    5,
                )
            ]
            routine["under right trench from start"] = [
                FollowTrajectory(
                    "under right trench from start",
                    isFlipped,
                )
            ]
            routine["right to balls"] = [
                FollowTrajectory(
                    "right to balls",
                    isFlipped,
                )
            ]
            routine["under left trench to hub"] = [
                FollowTrajectory(
                    "under left trench to hub",
                    isFlipped,
                )
            ]
            routine["shoot"] = [
                OperateTurret(
                    True,
                    8,
                )
            ]

        case AutoRoutines.A_EMPTY_AND_GET_BALLS_AND_SHOOT_FROM_LEFT:
            routine["shoot"] = [
                OperateTurret(
                    True,
                    5,
                )
            ]
            routine["under left trench from start"] = [
                FollowTrajectory(
                    "under left trench from start",
                    isFlipped,
                )
            ]
            routine["left to balls"] = [
                FollowTrajectory(
                    "left to balls",
                    isFlipped,
                )
            ]
            routine["under right trench to hub"] = [
                FollowTrajectory(
                    "under right trench to hub",
                    isFlipped,
                )
            ]
            routine["shoot"] = [
                OperateTurret(
                    True,
                    8,
                )
            ]

        case AutoRoutines.DROP_INTAKE:
            routine["Drop Intake"] = [
                OperateIntake(),
            ]

        case AutoRoutines.L_TEST:
            routine["L Test"] = [
                FollowTrajectory(
                    "l test",
                    isFlipped,
                ),
                OperateIntake(),
            ]

        case AutoRoutines.GET_BALLS_RIGHT_FROM_HUB:
            routine["Under Right Trench From Hub"] = [
                FollowTrajectory(
                    "under right trench from hub",
                    isFlipped,
                )
            ]
            routine["Right To Balls"] = [
                FollowTrajectory(
                    "right to balls",
                    isFlipped,
                ),
                OperateIntake(),
            ]
            routine["Under Left Trench To Hub"] = [
                FollowTrajectory(
                    "under left trench to hub",
                    isFlipped,
                )
            ]

        case AutoRoutines.GET_BALLS_LEFT_FROM_HUB:
            routine["Under Left Trench From Hub"] = [
                FollowTrajectory(
                    "under left trench from hub",
                    isFlipped,
                )
            ]
            routine["Left To Balls"] = [
                FollowTrajectory(
                    "left to balls",
                    isFlipped,
                ),
                OperateIntake(),
            ]
            routine["Under Right Trench To Hub"] = [
                FollowTrajectory(
                    "under right trench to hub",
                    isFlipped,
                )
            ]

        case AutoRoutines.GET_BALLS_RIGHT_FROM_START:
            routine["Under Right Trench From START"] = [
                FollowTrajectory(
                    "under right trench from start",
                    isFlipped,
                )
            ]
            routine["Right To Balls"] = [
                FollowTrajectory(
                    "right to balls",
                    isFlipped,
                ),
                OperateIntake(),
            ]
            routine["Under Left Trench To Hub"] = [
                FollowTrajectory(
                    "under left trench to hub",
                    isFlipped,
                )
            ]

        case AutoRoutines.GET_BALLS_LEFT_FROM_START:
            routine["Under Left Trench From START"] = [
                FollowTrajectory(
                    "under left trench from start",
                    isFlipped,
                )
            ]
            routine["Left To Balls"] = [
                FollowTrajectory(
                    "left to balls",
                    isFlipped,
                ),
                OperateIntake(),
            ]

            routine["Under Right Trench To Hub"] = [
                FollowTrajectory(
                    "under right trench to hub",
                    isFlipped,
                )
            ]

        case AutoRoutines.MESS_WITH_MIDDLE_FROM_RIGHT:
            routine["Under Right Trench From Hub"] = [
                FollowTrajectory(
                    "under right trench from hub",
                    isFlipped,
                )
            ]
            routine["Mess With Middle From Right"] = [
                FollowTrajectory(
                    "mess with middle from right",
                    isFlipped,
                ),
            ]
            routine["Under Left Trench To Hub"] = [
                FollowTrajectory(
                    "under left trench to hub",
                    isFlipped,
                )
            ]

        case AutoRoutines.FORWARD:
            routine["Forward"] = [
                FollowTrajectory(
                    "Drive Forward Test",
                    isFlipped,
                )
            ]

        case AutoRoutines.TEST_INTAKE:
            routine["Drop & Run Intake"] = [OperateIntake(10.5)]

        case AutoRoutines.DRIVE_FORWARD_BACK_TEST:
            routine["Forward"] = [
                FollowTrajectory(
                    "Forward",
                    isFlipped,
                ),
            ]
            routine["Backward"] = [
                FollowTrajectory(
                    "Backward",
                    isFlipped,
                )
            ]
        case AutoRoutines.WONKY:
            routine["Wonky 1"] = [
                FollowTrajectory(
                    "Wonky 1",
                    isFlipped,
                )
            ]
            routine["Wonky 2"] = [
                FollowTrajectory(
                    "Wonky 2",
                    isFlipped,
                )
            ]
        case AutoRoutines.FORWARD_AND_INTAKE:
            routine["Forward"] = [
                FollowTrajectory(
                    "Drive Forward Test",
                    isFlipped,
                ),
                OperateIntake(5),
            ]

        case AutoRoutines.FORWARD_AND_SHOOT:
            routine["Forward"] = [
                FollowTrajectory(
                    "Drive Forward Test",
                    isFlipped,
                )
            ]
            routine["shoot"] = [
                OperateTurret(
                    True,
                    0.5,
                )
            ]

        case AutoRoutines.JUST_SHOOT:
            routine["shoot"] = [OperateTurret(True, 15)]

    return routine
