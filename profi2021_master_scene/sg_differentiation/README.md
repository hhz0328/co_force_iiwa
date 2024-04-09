# Savitzky-Golay smoothing and differentiation
A package implementing Savitzky-Golay smoothing and differentiation. May be useful to estimate e.g. velocity and acceleration from position measurements.

**NOTE:** For Non-ROS users checkout the -cmake branch.

## Documentation
You can use doxygen to get some source code documentation which you may find useful when using the library. 

First install dixygen if you don't have it, then navigate to the sg_differentiation folder and run
```
doxygen Doxyfile
```

This will produce html files documenting the code. Open doc/html/index.html to browse them. 

## Usage
A minimalist cpp program using the filter could look like:

```c++
#include "eigen3/Eigen/Dense"
#include "sg_filter.h"

int main(int argc, char* argv[]){
  // for scalar data:
  int order = 2;
  int winlen = 11;
  SGF::real sample_time = 1e-3; // this is simply a float or double, you can change it in the header sg_filter.h if yo u want
  SGF::ScalarSavitzkyGolayFilter filter(order, winlen, sample_time);
  // add some data
  SGF::real new_data = GetYourSensorData() //put your raw data
  filter.AddData(new_data);
  // to get output:
  SGF::real output;
  int ret_code;
  ret_code = filter.GetOutput(0, output); // the 0 means we dont want differntiation. Put 1 to differentiate once, etc
  // ret_code will be -1 until at least winlen data points have been added, then it will be 0
  
  
  // multidim case
  int dim = 2;
  SGF::SavitzkyGolayFilter filter(dim, order, winlen, sample_time);
  SGF::Vec inp(dim); // SGF::Vec is the same as Eigen::Matrix<SGF::real, Eigen::Dynamic, 1>
  inp = GetYourSensorData();
  ret_code = filter.AddData(inp);
  // ret_code will be -2 if you try tp add data of wrong size
  
  SGF::Vec outp(dim);
  ret_code = filter.GetOutput(0, outp);
  // ret_code will be -1 until at least winlen data points have been added, then it will be 0
   
}


```
