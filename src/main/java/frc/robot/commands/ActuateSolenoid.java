// package frc.robot.commands;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class ActuateSolenoid extends CommandBase{
//     private DoubleSolenoid doubleSolenoid;
//     private Solenoid singleSolenoid;
//     private static boolean isDouble;
//     public ActuateSolenoid(int forward, int reverse) {
//         doubleSolenoid = new DoubleSolenoid(forward, reverse);
//         isDouble = true;
//     }

//     public ActuateSolenoid(int forward) {
//         singleSolenoid = new Solenoid(forward);
//         isDouble = false;
//     }

//     @Override
//     public void initialize() {
//         if (isDouble) {
//             doubleSolenoid.set(doubleSolenoid.get() == Value.kForward ? Value.kReverse : Value.kForward);
//         }
//         else {
//             singleSolenoid.set(!singleSolenoid.get());
//         }
//     }
// }