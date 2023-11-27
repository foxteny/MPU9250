#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "Filters.h"


MPU9250 accelgyro;
I2Cdev I2C_M;


//Set parameter
int16_t ax, ay, az;
int16_t gx, gy, gz;

float Axyz[3];
float Gxyz[3];

float preax = 0;
float preay = 0;
float preaz = 0;
float preax_dc = 0;
float preay_dc = 0;
float preaz_dc = 0;
float Ax_DC;
float Ay_DC;
float Az_DC;
float alpha = 0.6;
float filtered_ax;
float filtered_ay;
float filtered_az;
float pre_filtered_ax = 0;
float pre_filtered_ay = 0;
float pre_filtered_az = 0;
float Vx;
float Vy;
float Vz;
float preVx = 0;
float preVy = 0;
float preVz = 0;
float filtered_Vx;
float filtered_Vy;
float filtered_Vz;
float Vx_DC;
float Vy_DC;
float Vz_DC;
float prevx_DC = 0;
float prevy_DC = 0;
float prevz_DC = 0;
float Px;
float Py;
float Pz;
float prePx = 0;
float prePy = 0;
float prePz = 0;
float pre_filtered_Vx = 0;
float pre_filtered_Vy = 0;
float pre_filtered_Vz = 0;
float filtered_px;
float filtered_py;
float filtered_pz;
float first_position_x = 0;
float first_position_y = 0;
float first_position_z = 0;
int cnt = 0;

//Filter frequency
float filterFrequency = 0.1;
float cutoff_frequency = 1;

//Set filter
FilterOnePole highpassFilter_x( HIGHPASS, filterFrequency );
FilterOnePole highpassFilter_y( HIGHPASS, filterFrequency );
FilterOnePole highpassFilter_z( HIGHPASS, filterFrequency );

FilterOnePole lowpassFilter_x( LOWPASS, cutoff_frequency);
FilterOnePole lowpassFilter_y( LOWPASS, cutoff_frequency);
FilterOnePole lowpassFilter_z( LOWPASS, cutoff_frequency);

//
void setup() {
  Wire.begin();
  Serial.begin(9600);
  // Serial.println("Initializing I2C devices...");
  Serial.println("CLEARSHEET"); // clears sheet starting at row 1
  accelgyro.initialize();

  // Serial.println("Testing device connections...");
  // Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  Serial.println("LABEL,Ax,Ay,Az");
  delay(1000);

}

void loop() {
  cnt = cnt+1;
  getAccel_Data();
  getGyro_Data();
  Accel_raw_to_DC();
  filteredaccel();
  velocity();
  velocity_filter();
  position();
  velocity_DC();
  position_filter();

  // Serial.print("Data,      ");
  // Serial.println("Raw_Data / DC_Block_Data");
  // Serial.println("Ax, Ay, Az, DC_Ax, DC_Ay, DC_Az");
  // Serial.print(Axyz[0],4);
  // Serial.print(",     ");
  // Serial.print(Axyz[1],4);
  // Serial.print(",     ");
  // Serial.println(Axyz[2],4);
  // Serial.print(",");
  // Serial.print(Gxyz[0],4);
  // Serial.print(",");
  // Serial.print(Gxyz[1],4);
  // Serial.print(",");
  // Serial.println(Gxyz[2],4);
  // Serial.println(",    ");
  // Serial.print(Ax_DC,4);
  // Serial.print(",    ");
  // Serial.print(Ay_DC,4);
  // Serial.print(",    ");
  // Serial.println(Az_DC,4);
  preax = Axyz[0];
  preay = Axyz[1];
  preaz = Axyz[2];
  // Serial.println("DC and Filtered data");
  // Serial.print(filtered_ax,4);
  // Serial.print(",    ");
  // Serial.print(filtered_ay,4);
  // Serial.print(",    ");
  // Serial.println(filtered_az,4);
  // Serial.print(Vx,4);
  // Serial.print(",    ");
  // Serial.print(Vy,4);
  // Serial.print(",    ");
  // Serial.println(Vz,4);
  // Serial.print(Vx_DC,4);
  // Serial.print(",    ");
  // Serial.print(Vy_DC,4);
  // Serial.print(",    ");
  // Serial.println(Vz_DC,4);
  Serial.print(filtered_Vx,4);
  Serial.print(",    ");
  Serial.print(filtered_Vy,4);
  Serial.print(",    ");
  Serial.println(filtered_Vz,4);
  // Serial.print(Px,4);
  // Serial.print(",    ");
  // Serial.print(Py,4);
  // Serial.print(",    ");
  // Serial.println(Pz,4);
  // Serial.print(filtered_px,4);
  // Serial.print(",    ");
  // Serial.print(filtered_py,4);
  // Serial.print(",    ");
  // Serial.println(filtered_pz,4);
  

  preax_dc = Ax_DC;
  preay_dc = Ay_DC;
  preaz_dc = Az_DC;
  pre_filtered_ax = filtered_ax;
  pre_filtered_ay = filtered_ay;
  pre_filtered_az = filtered_az;
  preVx = Vx;
  preVy = Vy;
  preVz = Vz;
  prePx = Px;
  prePy = Py;
  prePz = Pz;
  pre_filtered_Vx = filtered_Vx;
  pre_filtered_Vy = filtered_Vy;
  pre_filtered_Vz = filtered_Vz;
  prevx_DC = Vx_DC;
  prevy_DC = Vy_DC;
  prevz_DC = Vz_DC;
  // delay(100);

}


void getAccel_Data(void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Axyz[0] = (double) ax/16384;
  Axyz[1] = (double) ay/16384;
  Axyz[2] = (double) az/16384;

  // Axyz[0] = Axyz[0]*9.8;
  // Axyz[1] = Axyz[1]*9.8;
  // Axyz[2] = Axyz[2]*9.8;
}

void getGyro_Data(void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Gxyz[0] = (double) gx/32768;
  Gxyz[1] = (double) gy/32768;
  Gxyz[2] = (double) gz/32768;
}

//Use DC Block Filter
void Accel_raw_to_DC(void)
{
  Ax_DC = Axyz[0] - preax + (alpha*preax_dc);
  Ay_DC = Axyz[1] - preay + (alpha*preay_dc);
  Az_DC = Axyz[2] - preaz + (alpha*preaz_dc);
  if (cnt < 100)
  {
    Ax_DC = 0;
    Ay_DC = 0;
    Az_DC = 0;
  }
}

//Filtering
void filteredaccel(void)
{
  // highpassFilter_x.input(Ax_DC);
  // highpassFilter_y.input(Ay_DC);
  // highpassFilter_z.input(Az_DC);
  // filtered_ax = highpassFilter_x.output();
  // filtered_ay = highpassFilter_y.output();
  // filtered_az = highpassFilter_z.output();

  lowpassFilter_x.input(Ax_DC);
  lowpassFilter_y.input(Ay_DC);
  lowpassFilter_z.input(Az_DC);

  
  filtered_ax = lowpassFilter_x.output();
  filtered_ay = lowpassFilter_y.output();
  filtered_az = lowpassFilter_z.output();

  // filtered_ax *= 9.8;
  // filtered_ay *= 9.8;
  // filtered_az *= 9.8; //단위 g -> m/s^2
  // if (filtered_ax >=0) {
  //   filtered_ax = ((filtered_ax*100)-((filtered_ax*100) -  floor(filtered_ax*100)))/100;// raw data - raw data의 3번째 아래 값들
  // }
  // else{
  //   filtered_ax =  ((filtered_ax*100) - ((filtered_ax*100) +floor(filtered_ax * 100 * (-1))))/100;
  // }
  // if (filtered_ay >=0) {
  //   filtered_ay = ((filtered_ay*100)-((filtered_ay*100) -  floor(filtered_ay*100)))/100;// raw data - raw data의 3번째 아래 값들
  // }
  // else{
  //   filtered_ay =  ((filtered_ay*100) - ((filtered_ay*100) +floor(filtered_ay * 100 * (-1))))/100;
  // }
  // if (filtered_az >=0) {
  //   filtered_az = ((filtered_az*100)-((filtered_az*100) -  floor(filtered_az*100)))/100;// raw data - raw data의 3번째 아래 값들
  // }
  // else{
  //   filtered_az =  ((filtered_az*100) - ((filtered_az*100) +floor(filtered_az * 100 * (-1))))/100;
  // }
  
}


//Just accel to Velocity
// void velocity(void)
// {
//   Vx = preVx + (Axyz[0] + preax) * 0.005;
//   Vy = preVy + (Axyz[1] + preay) * 0.005;
//   Vz = preVz + (Axyz[2] + preaz) * 0.005;
// }
//Just DC Block Filter accel to Velocity
// void velocity(void)
// {
//   Vx = preVx + (Ax_DC + preax_dc) * 0.005;
//   Vy = preVy + (Ay_DC + preay_dc) * 0.005;
//   Vz = preVz + (Az_DC + preaz_dc) * 0.005;
// }


//Filtered accel to Velocity
void velocity(void)
{
  Vx = preVx + (filtered_ax + pre_filtered_ax) * 0.005;
  Vy = preVy + (filtered_ay + pre_filtered_ay) * 0.005;
  Vz = preVz + (filtered_az + pre_filtered_az) * 0.005;
}
//Use DC Block Filter 
void velocity_DC(void)
{
  Vx_DC = Vx - preVx + (alpha * prevx_DC);
  Vy_DC = Vy - preVy + (alpha * prevy_DC);
  Vz_DC = Vz - preVz + (alpha * prevz_DC);
}

// void velocity_filter(void)
// {
//   // highpassFilter_x.input(Vx);
//   // highpassFilter_y.input(Vy);
//   // highpassFilter_z.input(Vz);
//   lowpassFilter_x.input(Ax_DC);
//   lowpassFilter_y.input(Ay_DC);
//   lowpassFilter_z.input(Az_DC);


//   // filtered_Vx = highpassFilter_x.output();
//   // filtered_Vy = highpassFilter_y.output();
//   // filtered_Vz = highpassFilter_z.output();
//   filtered_Vx = lowpassFilter_x.output();
//   filtered_Vy = lowpassFilter_y.output();
//   filtered_Vz = lowpassFilter_z.output();
// }

//Velocity Filtering
void velocity_filter(void)
{
  highpassFilter_x.input(Vx);
  highpassFilter_y.input(Vy);
  highpassFilter_z.input(Vz);
  // lowpassFilter_x.input(Vx);
  // lowpassFilter_y.input(Vy);
  // lowpassFilter_z.input(Vz);


  filtered_Vx = highpassFilter_x.output();
  filtered_Vy = highpassFilter_y.output();
  filtered_Vz = highpassFilter_z.output();
  // lowpassFilter_x.input(filtered_Vx);
  // lowpassFilter_y.input(filtered_Vy);
  // lowpassFilter_z.input(filtered_Vz);
  // filtered_Vx = lowpassFilter_x.output();
  // filtered_Vy = lowpassFilter_y.output();
  // filtered_Vz = lowpassFilter_z.output();
  // if (filtered_Vx >=0) {
  //   filtered_Vx = (filtered_Vx*100)-((filtered_Vx*100) -  floor(filtered_Vx*100));// raw data - raw data의 3번째 아래 값들
  // }
  // else{
  //   filtered_Vx =  (filtered_Vx*100) - ((filtered_Vx*100) +floor(filtered_Vx * 100 * (-1)));
  // }
  // if (filtered_Vy >=0) {
  //   filtered_Vy = (filtered_Vy*100)-((filtered_Vy*100) -  floor(filtered_Vy*100));// raw data - raw data의 3번째 아래 값들
  // }
  // else{
  //   filtered_Vy =  (filtered_Vy*100) - ((filtered_Vy*100) +floor(filtered_Vy * 100 * (-1)));
  // }
  // if (filtered_Vz >=0) {
  //   filtered_Vz = (filtered_Vz*100)-((filtered_Vz*100) -  floor(filtered_Vz*100));// raw data - raw data의 3번째 아래 값들
  // }
  // else{
  //   filtered_Vz =  (filtered_Vz*100) - ((filtered_Vz*100) +floor(filtered_Vz * 100 * (-1)));
  // }
}

//DC Filter Velocity to Position
// void position(void)
// {
//   Px = prePx + (Vx_DC + prevx_DC)*0.005;
//   Py = prePy + (Vy_DC + prevy_DC)*0.005;
//   Pz = prePz + (Vz_DC + prevz_DC)*0.005;
// }

//Row Velocity to Position
// void position(void)
// {
//   Px = prePx + (Vx + preVx)*0.005;
//   Py = prePy + (Vy + preVy)*0.005;
//   Pz = prePz + (Vz + preVz)*0.005;
// }

//Filtered Velocity to Position
void position(void)
{
  Px = prePx + (filtered_Vx + pre_filtered_Vx)*0.005;
  Py = prePy + (filtered_Vy + pre_filtered_Vy)*0.005;
  Pz = prePz + (filtered_Vz + pre_filtered_Vz)*0.005;
}


//Filtering
void position_filter(void)
{
  highpassFilter_x.input(Px);
  highpassFilter_y.input(Py);
  highpassFilter_z.input(Pz);

  filtered_px = highpassFilter_x.output();
  filtered_py = highpassFilter_y.output();
  filtered_pz = highpassFilter_z.output();
}
