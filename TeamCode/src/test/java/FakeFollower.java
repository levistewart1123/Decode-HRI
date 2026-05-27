public class FakeFollower {
    double f, r, t;
    boolean teleOpDriveStarted = false;
    public void setTeleOpDrive(double f, double r, double t){
        this.f = f;
        this.r = r;
        this.t = t;
    }
    public void startTeleOpDrive(){
        teleOpDriveStarted = true;
    }

}
