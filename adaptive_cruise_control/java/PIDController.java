/*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

package pid;

// This code implements a PID Controller
// =====================================
 

public class PIDController 
{
    // Kp -  proportional gain
	double Kp;
    // Ki -  Integral gain
	double Ki;
    // Kd -  derivative gain
	double Kd;
    // dt -  loop interval time
	double dt;
    // max - maximum value of manipulated variable
	double max;
    // min - minimum value of manipulated variable
	double min;
	// pre_error - pre error
	double pre_error;
	// integral - integral
	double integral;
	
	public PIDController(double dt, double max, double min, double Kp, double Kd, double Ki )
	{
		this.dt = dt;
		this.max = max;
		this.min = min;
		this.Kp = Kp;
		this.Kd = Kd;
		this.Ki = Ki;
		this.pre_error = 0.0;
		this.integral = 0.0;
	}
	
	public double compute(double setpoint, double pv)
	{
		// Calculate error
	    double error = setpoint - pv;

	    // Proportional term
	    double Pout = Kp * error;

	    // Integral term
	    integral += error * dt;
	    double Iout = Ki * integral;

	    // Derivative term
	    double derivative = (error - pre_error) / dt;
	    double Dout = Kd * derivative;

	    // Calculate total output
	    double output = Pout + Iout + Dout;

	    // Restrict to max/min
	    if( output > max )
	        output = max;
	    else if( output < min )
	        output = min;

	    // Save error to previous error
	    pre_error = error;

	    return output;
	}
	
	public static void main(String[] args) 
	{
	   PIDController pid = new PIDController(0.1, 100, -100, 0.1, 0.01, 0.5);

	    double val = 20;
	    for (int i = 0; i < 1000; i++) 
	    {
	        double inc = pid.compute(0, val);
	        System.out.println("val: " + val + " inc: " + inc);
	        val += inc;
	    }
	}
}
