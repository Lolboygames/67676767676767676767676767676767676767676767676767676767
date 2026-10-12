use vexide::prelude::*;
use vexide::smart::motor::BrakeMode;
use vexide_motorgroup::MotorGroup;
use std::time::Duration;
use evian::prelude::*;

use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::{Arcade, Differential},
    motion::{Basic, Seeking},
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};





struct Robot {
    drivetrain: Drivetrain<Differential, WheeledTracking>,
    controller: Controller,
}

impl Robot {
    const Linear: Pid = Pid::new(1.0,0.0,0.125, None);
    const Angular: AngularPid =  AngularPid::new(16.0, 0.0, 1.0, None);
    const LinTol: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.085)
        .duration(Duration::from_millis(10));
    const AngTol: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.085)
        .duration(Duration::from_millis(15));
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let dt = &mut self.drivetrain;
        let mut seeking = Seeking {
            linear_controller: Pid::new(1.0,0.0,0.125, None),
            lateral_controller: Pid::new(1.0,0.0,0.125, None),
            tolerances: Self::LinTol,
            timeout: Some(Duration::from_millis(10)),
        };
        let mut basic = Basic {
            linear_controller: Self::Linear,
            angular_controller: Self::Angular,
            angular_tolerances: Self::AngTol,
            linear_tolerances: Self::LinTol,
            timeout: Some(Duration::from_millis(10)),
        };
        seeking.move_to_point(dt,(24.0, 24.0)).await;





    }
    async fn driver(&mut self) {
        loop{
            let state = self.controller.state().unwrap_or_default();
            _= self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.right_stick.x());


            println!("{}", self.drivetrain.tracking.position());
            sleep(Duration::from_millis(10000)).await;

        }
    }

}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    //Declare Motors
    let mut lm1 = Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward);
    let mut lm2 = Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse);
    let mut hlm = Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse);
    let mut rm1 = Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward);
    let mut rm2 = Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward);
    let mut hrm = Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse);
    // Declare Left Motor Group
    let lm = [
        lm1,
        lm2,
        hlm
    ];
    // Declare Right Motor Group
    let rm = [
        rm1,
        rm2,
        hrm
    ];

    let controller = peripherals.primary_controller;
    //Define Tracking Wheels
    let vodomrot = RotationSensor::new(peripherals.port_7, Direction::Forward);
    //let Vodom = TrackingWheel::new(vodomrot, 2.0, 1.5, None);
    let horizrot = RotationSensor::new(peripherals.port_20, Direction::Reverse);
    //let Hodom = TrackingWheel::new(horizrot, 2.0, 2.0, None);
    //Defiine IMU
    let mut imu = InertialSensor::new(peripherals.port_13);
    //imu.calibrate().await.unwrap();

    Robot{
        drivetrain: Drivetrain::new(
            Differential::new(lm,rm),
            WheeledTracking::new(
                (0.0,0.0),
                90.0.deg(),
                [TrackingWheel::new(vodomrot,2.0, 1.5, None)],
                [TrackingWheel::new(horizrot,2.0, 1.5, None)],
                Some(imu),
            ),

        ),
        controller,
    }
    .compete()
        .await;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_adds_two() {
        assert_eq!(2 + 2, 4);   
    }
}
