import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ControllerInputHandler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main", group = "TeleOp")
public class RyanMain extends OpMode {
    private ControllerInputHandler controllerInput;
    private DcMotor leftMotorfront;
    private DcMotor rightMotorfront;
    private DcMotor leftMotorback;
    private DcMotor rightMotorback;
    private DcMotor hdHexMotor;


    @Override
    public void init() {
        controllerInput = new ControllerInputHandler(gamepad1);
        /*
        coreHexMotorClass = new MotorRun(coreHexMotor, 0, "forward"); // power, direction
        coreHexMotor = hardwareMap.get(DcMotor.class, "coreHexMotor");
        coreHexMotor.setDirection(DcMotorSimple.Direction.FORWARD);*/

        leftMotorfront = hardwareMap.get(DcMotor.class, "left");
        leftMotorfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorfront.setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotorfront = hardwareMap.get(DcMotor.class, "right");
        rightMotorfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorfront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotorback = hardwareMap.get(DcMotor.class, "left");
        leftMotorback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorback.setDirection(DcMotorSimple.Direction.FORWARD);

        rightMotorback = hardwareMap.get(DcMotor.class, "right");
        rightMotorback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorback.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {

    }
    public void forward()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(speed);
        rightMotorback.setPower(speed);
    }

    public void moveright()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(-speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(speed);
    }

    public void moveleft()
    {
        leftMotorfront.setPower(-speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(speed);
        rightMotorback.setPower(-speed);
    }
    public void backward()
    {
        leftMotorfront.setPower(-speed);
        leftMotorback.setPower(-speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);
    }

    public void diagonalfright()
    {
        leftMotorfront.setPower(speed);
        rightMotorback.setPower(speed);
    }
    public void diagonalfleft()
    {
        rightMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
    }
    public void diagonalbright()
    {
        leftMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);
    }
    public void diagonalbleft()
    {
        rightMotorfront.setPower(-speed);
        leftMotorback.setPower(-speed);
    }

    public void backturn()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);
    }
    public void leftturn()
    {
        leftMotorfront.setPower(-speed);
        leftMotorback.setPower(-speed);
        rightMotorfront.setPower(speed);
        rightMotorback.setPower(speed);
    }
    public void rightturn()
    {
        leftMotorfront.setPower(speed);
        leftMotorback.setPower(speed);
        rightMotorfront.setPower(-speed);
        rightMotorback.setPower(-speed);


}


