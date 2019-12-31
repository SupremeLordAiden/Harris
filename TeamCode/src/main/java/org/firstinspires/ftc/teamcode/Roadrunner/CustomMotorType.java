package org.firstinspires.ftc.teamcode.Roadrunner;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev=383.6, gearing=13.7, maxRPM=435, orientation=Rotation.CCW)
@DeviceProperties(xmlTag="goBILDA5202SeriesMotor435RPM", name="GoBILDA 5202 series 435 RPM", builtIn = true)
@DistributorInfo(distributor="goBILDA_distributor", model="goBILDA-5202-435", url="https://www.gobilda.com/520x2-series-yellow-jacket-planetary-gear-motors/")
@ExpansionHubPIDFVelocityParams(P=2.0, I=0.5, F=11.1)
@ExpansionHubPIDFPositionParams(P=5.0)
public interface CustomMotorType
{
}