#ifndef CONTROL_H
#define CONTROL_H

//---------------aperiodic--------------------------------
class aperiodic
{
public:
  float T = 0.0;
  float k = 0.0;
  float dt = 0.0;
  float y0 = 0.0;

  void set_values(float T, float k, float dt, float y0)
  {
    this->T = T;
    this->k = k;
    this->dt = dt;
    this->y0 = y0;
  }

  float aperiodic_solver(float u)
  {
      y0 = y0 + dt*(1/T) * (k*u-y0);
      return y0;
  }
};
//-------------------------------------------------------------------

//---------------- integral -----------------------------------------
class integral
{
public:
    float dt = 0.0;
    float y0 = 0.0;

    void set_values(float dt, float y0)
    {
        this->dt = dt;
        this->y0 = y0;
    }

    void integral_solver(float u)
    {
        y0 = y0 + dt * u;
    }
};
//-------------------------------------------------------------------

//---- Медианный (медиана на 3 значения со своим буфером) -----------
class median
{
public:
    float median_solver(float newVal) {
    static float buf[3];
    static byte count = 0;
    buf[count] = newVal;
    if (++count >= 3) count = 0;
    return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
    }
};
//-------------------------------------------------------------------

//---------------- Экспоненциальное бегущее среднее -----------------
class expRunningAverage
{
public:
    float k = 0.0;  // коэффициент фильтрации, 0.0-1.0
    float filVal = 0.0;
    
    void set_values(float k, float filVal)
    {
        this->k = k;
        this->filVal = filVal;
    }
    
    float expRunningAverage_solver(float newVal) {
    filVal += (newVal - filVal) * k;
    return filVal;
    }
};
//-------------------------------------------------------------------

//randomizer
/*float rand_noize(float u)
{
    if (u == 0)
    {
        return (float)(rand() - RAND_MAX * 0.5) / (RAND_MAX * 0.5);
    }
    else
    {
        float v = (float)(rand() - RAND_MAX * 0.5) / (RAND_MAX * 0.5);

        return v * 0.05 * u;
    }  
}*/

//---------------- primary filter -----------------------------------
/*double primary_filter(std::array<double, 8> &f, double new_val)
{
    //shit code, but otherwise it is difficult
    double sum = 0;

    std::array<double, 8> f2;

    for (int i = 0; i < f.size(); i++)
    {
        f2[i] = f[i];
    }

    for (int i = f.size() - 2; i >= 0; i--)
    {
        f[i + 1] = f[i];
    }

    // add new value
    f[0] = new_val;

    //sort array
    std::sort(f2.begin(), f2.end());

    for (int i = 2; i < f2.size()-2; i++)
    {
        sum += f2[i];
    }

    return sum / (f2.size() - 4);
}*/
//-------------------------------------------------------------------

//---------------- deadzone -----------------------------------------
float deadzone(float u, float zone)
{
    if (abs(u) < zone) u = 0;
    return u;
}
//-------------------------------------------------------------------

#endif // CONTROL_H
