void setup() {
  // put your setup code here, to run once:
int A = 6; 
//adding more comments to demonstrate
}

void loop() {
  short PI_controller(double e_now, double e_int, double k_P, double k_I)
  {
    short = u; 
    u = short(k_P*e_now+k_I*e_int); 
    if (u>255)
    {
      u = 255;
    }
    else if (u<-255)
    {
      u = -255;
    }
    return u; 
  }
  u_L = k_P(v_l)

}
