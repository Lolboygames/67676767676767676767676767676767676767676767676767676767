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
    drivetrain: Drivetrain<Differential, ()>,
    controller: Controller,
    lift1: Motor,
    lift2: Motor,
    claw: AdiDigitalOut,
}

impl Robot {
    const Linpid: Pid = Pid::new(1.0,0.0,0.125, None);
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
            timeout: Some(Duration::from_secs(10)),
        };
        let mut basic = Basic {
            linear_controller: Self::Linpid,
            angular_controller: Self::Angular,
            angular_tolerances: Self::AngTol,
            linear_tolerances: Self::LinTol,
            timeout: Some(Duration::from_secs(10)),
        };






    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();
            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.right_stick.x());

            sleep(Duration::from_millis(10)).await;

                if state.button_a.is_now_pressed() {

                    if self.claw.is_high().unwrap_or_default() {
                        self.claw.set_low().unwrap();
                    } else {
                        self.claw.set_high().unwrap();
                    }
                }

                if state.button_r1.is_pressed() {
                    self.lift1.set_voltage(12.7).ok();
                    self.lift2.set_voltage(12.7).ok();
                } else {
                    if state.button_r2.is_pressed() {
                        self.lift1.set_voltage(-12.7).ok();
                        self.lift2.set_voltage(-12.7).ok();
                    } else {
                        self.lift1.brake(BrakeMode::Brake).ok();
                        self.lift2.brake(BrakeMode::Brake).ok();
                    }


            }


            }


            //println!("{}", self.drivetrain.tracking.position());

        }
    }





#[vexide::main]
async fn main(peripherals: Peripherals) {
    //Declare Motors
    //let vodomrot = RotationSensor::new(peripherals.port_7, Direction::Forward);
    //let Vodom = TrackingWheel::new(vodomrot, 2.0, 1.5, None);
    //let horizrot = RotationSensor::new(peripherals.port_20, Direction::Reverse);
    //let Hodom = TrackingWheel::new(horizrot, 2.0, 2.0, None);
    //Defiine IMU
    //let mut imu = InertialSensor::new(peripherals.port_13);
    let mut lm1 = Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse);
    let mut lm2 = Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse);
    let mut hlm = Motor::new_exp(peripherals.port_3, Direction::Reverse);
    let mut rm1 = Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward);
    let mut rm2 = Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward);
    let mut hrm = Motor::new_exp(peripherals.port_6, Direction::Forward);
    let mut lift1 = Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward);
    let mut lift2 = Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward);
    let mut claw = AdiDigitalOut::new(peripherals.adi_a);

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

    Robot {
        drivetrain: Drivetrain::new(
            Differential::new(lm, rm),
            (),
        ),
        controller: peripherals.primary_controller,

        lift1: lift1,

        lift2: lift2,
        claw: claw,




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


