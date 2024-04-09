#ifndef SG_FILTER_H
#define SG_FILTER_H

#include <boost/circular_buffer.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>

namespace SGF{

  typedef float real;
  typedef Eigen::Matrix<real, Eigen::Dynamic, 1> Vec;
  typedef Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic> Mat;
  
  
  

  /*
    Return error values:

    -1: you tried to get output before the filter was initialized. The filter is initialized by adding winlen data points before querying for outputs. Truncating with the first value is done, but you should be aware that the output is not reliable, especially for derivatives. 

    -2: you tried to add data of incorrect dimension. 

  */


  /**
     \brief This class implements savitzky-golay filtering for online use, assuming a fixed sampling period. This class only supports scalar data. 

     Detailed description starts here
  */

  class ScalarSavitzkyGolayFilter{
  private:
    int winlen_;
    int order_;
    real * data_buffer_;
    int n_added_;
    real sample_time_;

    Mat A_;


    Vec FitCoeffs();


  public:
    /**
       \brief Constructor. The order is the order of the interpolating polynomial to be used. The number of samples to be interpolated is given by the paramter winlen. The sample period, which should be the same between each sample, is given in sample_time. 

       In general, the higher the winlen the smoother the results, but also the more delayed the signals will be. The winlen should be at least order+1 but preferably a much bigger number than order. Good values depends from application to application, but you could start with e.g. 
       order 2, winlen 11
    */
    ScalarSavitzkyGolayFilter(int order,int winlen, real sample_time);


    /** \brief  convenience function combining a call to AddData and GetOutput(diff_order, output)
	@return
	0: everything ok \n
	-1: filter is not initialized, use with caution!

    */
    int Filter(real input, int diff_order, real & output);

    /**
       \brief Compute filtered output. 

       The forward_param determines how far from the center the polynomial should be queried. The default is forward_param=0 which would imply  a delay of winlen/2 * sample_time. To reduce the delay, you may use a forward_param in the interval [0, 1], where 1 corresponds to going to the very end of the range used for fitting the polynomial (Not recommended!). A forward_param>0 will reduce the delay but ther is no free lunch - the quality of the ouput will not be as good. 

       @return
       0: everything ok \n
       -1: filter is not initialized, use with caution!
    */
  
    int GetOutput(real forward_param,int diff_order, real & output);

    /**
       \brief Compute filtered output. 

       Get output with forward_param = 0.0. Standard Savitzky-Golay filtering. 
       @return
       0: everything ok \n
       -1: filter is not initialized, use with caution!

    */

    int GetOutput(int diff_order, real & output);
    /**
       \brief Add new data to the filter. 

       Adds a new data point to the filter data buffer. 
       @return
       0: everything ok

    */
    int AddData(real new_data);


    /**
       \brief Check wether the filter has enough data to give reliable results. 

       The filter is only reliable if at least winlen number of data has been added. 
    */

    bool IsInitialized();

    // internal non-documented functions only for testing purposes
    Mat GetPolynomialMatrix(){
      return A_;
    }

    Eigen::Map<Vec> GetData(){
      return Eigen::Map<Vec>(data_buffer_,winlen_);
    }

  };


  inline  Vec diff_poly_coeffs(Vec coeffs,int n_diff){
    if(n_diff == 0)
      return coeffs;
    for(int i=0; i<coeffs.size(); i++)
      coeffs[i] = coeffs[i]*i;
    return diff_poly_coeffs(coeffs.tail(coeffs.size()-1), n_diff-1);
  }



  /**
     \brief Multidimensional Savitzky-Golay filter for online use. This is a very simple implementation with several ScalarSavitzkyGolay filters running in paralell. This implementation is very far from being optimal. 

  */


  class SavitzkyGolayFilter{
  private:
    std::vector<ScalarSavitzkyGolayFilter> scalar_filters_;
    int dim_;
  public:
    /**
       \brief Constructor. Provide the dimensionality of the data that you want to filter, the order of the interpolating polynomials and the number of data to considered when fitting each polynomial (winlen). 

       In general, the higher the winlen the smoother the results, but also the more delayed the signals will be. The winlen should be at least order+1 but preferably a much bigger number than order. Good values depends from application to application, but you could start with e.g. 
       order 4, winlen 31
    */

    SavitzkyGolayFilter(int dim, int order, int winlen, real sample_time);
    /**
       \brief Cheecks if the filter is initialized. 

       The filter will always provide an output if you ask it for one, however it is only reliable once at least winlen data points have been added. This function can be used to let you know if enough data has been added. 
    */

    bool IsInitialized();

    /**
       \brief Adds a new data point to the filter. 

       @return
       0: everything ok \n
       -2: you tried to add data with incorrect number of dimensions. 
    */

    int AddData(const Vec & inp);
    /**
       \brief Compute filtered output. 

       Get output with forward_param = 0.0. Standard Savitzky-Golay filtering. 
       @return
       0: everything ok \n
       -1: filter is not initialized, use with caution!

    */


    int GetOutput(int diff_order, Vec & result);
    /**
       \brief Compute filtered output. 

       The forward_param determines how far from the center the polynomial should be queried. The default is forward_param=0 which would imply  a delay of winlen/2 * sample_time. To reduce the delay, you may use a forward_param in the interval [0, 1], where 1 corresponds to going to the very end of the range used for fitting the polynomial (Not recommended!). A forward_param>0 will reduce the delay but ther is no free lunch - the quality of the ouput will not be as good. 

       @return
       0: everything ok \n
       -1: filter is not initialized, use with caution!
       -3: no data has been added, no result computed
    */

    int GetOutput(real forward_param, int diff_order, Vec & result);

    /** \brief  convenience function combining a call to AddData and GetOutput(diff_order, output)
	@return
	0: everything ok \n
	-1: filter is not initialized, use with caution!
	-3: no data has been added, no result computed

    */
    int Filter(const Vec & input, int diff_order, Vec & output);

  
  };

};
#endif //SG_FILTER_H
