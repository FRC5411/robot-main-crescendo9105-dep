package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HolonomicController;
import frc.robot.util.SimpleUtils;
import frc.robot.util.HolonomicController.HolonomicConstraints;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class moveToPosition {
    private Supplier<Pose2d> currentPose;
    private Supplier<ChassisSpeeds> currentChassisSpeeds;
    private Consumer<ChassisSpeeds> setDesiredStates;
    private Field2d field;
    private Subsystem requirements;
    private VisionSubsystem vision;
    private Pose2d target = new Pose2d();

    public moveToPosition( 
        Supplier<Pose2d> curPoseSupplier, Supplier<ChassisSpeeds> curSpeedSupplier, Field2d field,
        Consumer<ChassisSpeeds> setDesiredStates, Subsystem requirements, VisionSubsystem vision) {
        currentPose = curPoseSupplier;
        currentChassisSpeeds = curSpeedSupplier;
        this.setDesiredStates = setDesiredStates;
        this.requirements = requirements;
        this.vision = vision;
    }

    // Timed Commands
    public Command generateMoveToPositionCommandTimed(Pose2d targetPose, Pose2d tolerance, 
            HolonomicConstraints profiles, HolonomicController controller ) {
        return generateMoveToPositionCommandTimed(targetPose, new ChassisSpeeds(), tolerance, profiles, controller);
    }
    
    public Command generateMoveToPositionCommandTimed( Pose2d targetPose, ChassisSpeeds targetChassisSpeeds, 
            Pose2d tolerance,  HolonomicConstraints profiles, HolonomicController controller) {
            Telemetry.setValue("Alignment/MOM", profiles.getLongestTime(targetPose, targetChassisSpeeds));
        return generateMoveToPositionCommand(targetPose, targetChassisSpeeds, tolerance, controller)
            .withTimeout(profiles.getLongestTime(targetPose, targetChassisSpeeds) + 0.5);
    }

    // Regular Commands
    public Command generateMoveToPositionCommand( 
        Pose2d targetPose, Pose2d tolerance, HolonomicController controller ) {
        return generateMoveToPositionCommand( 
            targetPose, 
            new ChassisSpeeds(), 
            tolerance, 
            controller );
    }

    public Command generateMoveToPositionCommand(
        Pose2d targetPose, ChassisSpeeds targetChassisSpeeds, 
        Pose2d tolerance, HolonomicController controller ) {
        final double xKi = controller.getXController().getI();
        final double yKi = controller.getYController().getI();
        final double thetaKi = controller.getThetaController().getI();

        return new FunctionalCommand(
            () -> {
                vision.setPipelineIndices(0);
                this.target = targetPose;
                controller.setTolerance( tolerance );
                controller.reset( currentPose.get(), currentChassisSpeeds.get() );
                controller.setGoal(targetPose, targetChassisSpeeds);
                
                field.getObject( "Goal" ).setPose( controller.getPositionGoal() );
            },
            () -> {
                controller.xIZone(xKi, currentPose.get(), 1);
                controller.yIZone(yKi, currentPose.get(), 0.5);
                controller.thetaIZone(thetaKi, currentPose.get(), 5);

                setDesiredStates.accept( SimpleUtils.discretize( controller.calculateWithFF( currentPose.get() ) ) );

                field.getObject( "Setpoint" ).setPose( controller.getPositionSetpoint() );

                if(RobotBase.isSimulation()) requirements.resetPose( controller.getPositionSetpoint() );
            }, 
            (interrupted) -> { requirements.joystickDrive(0, 0, 0); }, 
            () -> controller.atGoal(),
            requirements) ;
    }

    public Pose2d getTarget() {
        return target;
    }
}