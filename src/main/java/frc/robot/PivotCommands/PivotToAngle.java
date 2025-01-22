package frc.robot.PivotCommands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
 
public class PivotToAngle extends Command{

    pivot s_Pivot = new pivot();
    double setPoint;

    public PivotToAngle(pivot pivot, double setPoint)
    {
        s_Pivot = pivot;
        addRequirements(s_Pivot);
    }

    public void execute() 
    {
        //pivots arm to setPoint
        s_Pivot.goToSetPoint(setPoint);
        
        if (s_Pivot.pivotMaster.getPosition().getValueAsDouble() <setPoint-0.19)
        {
          return;
        }
    }
}
