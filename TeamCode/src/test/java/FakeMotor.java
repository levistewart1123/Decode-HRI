public class FakeMotor {
    double currentPower = 0;
    public void set(double power){
        currentPower = power;
    }
    public double get() {
        return currentPower;
    }
}
