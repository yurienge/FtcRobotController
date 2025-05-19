package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Cortex_yuri", group ="Linear OpMode")
public abstract class Cortex_yuri extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL0 = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;

    @Override
    public void runOpMode() {

        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");

        FL0.setDirection(DcMotor.Direction.REVERSE);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        BR3.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData ("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.left_stick_x;

            double FL0 = axial + lateral + yaw;
            double FR1 = axial - lateral - yaw;
            double BL2 = axial - lateral + yaw;
            double BR3 = axial + lateral - yaw;

            max = Math.max(Math.abs(FL0), Math.abs(FR1));
            max = Math.max(max, Math.abs(BL2));
            max = Math.max(max, Math.abs(BR3));

            if (max > 1.0) {
                FL0 /= max;
                FR1 /= max;
                BL2 /= max;
                BR3 /= max;
            }

            /*
            FL0  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            FR1   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            BL2 = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            BR3  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
            FL0.setPower(FL0);
            FR1.setPower(FR1);
            BL2.setPower(BL2);
            BR3.setPower(BR3);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", FL0, FR1);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BL2, BR3);
            telemetry.update();
            }
        }}
