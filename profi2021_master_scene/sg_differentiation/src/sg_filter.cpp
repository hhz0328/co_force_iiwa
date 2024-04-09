#include "sg_filter.h"
#include "math.h"

using namespace SGF;

ScalarSavitzkyGolayFilter::ScalarSavitzkyGolayFilter(int order,int winlen, real sample_time): order_(order), winlen_(winlen), sample_time_(sample_time)
{
  n_added_ = 0;
  data_buffer_ = new real[winlen_];
  for(int i=0;i<winlen;i++)
    data_buffer_[i] = 0.0;
    A_.resize(winlen_, order_+1);
    for (int i=0; i<winlen_;i++){
        for(int j=0; j<order_+1; j++){
            A_(i,j) = pow((i-floor(winlen_/2)),j);
        }
    }
}

int ScalarSavitzkyGolayFilter::Filter(real input, int diff_order, real & output)
{
  AddData(input);
  return GetOutput(0, diff_order, output);
}

int ScalarSavitzkyGolayFilter::AddData(real new_data)
{
  // fill the buffer with the first data point, better than nothing
  if(n_added_ == 0){
    for(int i=0;i<winlen_;i++)
      data_buffer_[i] = new_data;
  }
  // shift the buffer and add the latest point at the end
  else{
    memmove(&data_buffer_[0], &data_buffer_[1], (winlen_-1)*sizeof(real));
    data_buffer_[winlen_-1] = new_data;
    // memmove(&data_buffer_[1], &data_buffer_[0], (winlen_-1)*sizeof(real));
    // data_buffer_[0] = new_data;
  }
  n_added_++;
  return 0;
}

Vec ScalarSavitzkyGolayFilter::FitCoeffs()
{
  // an Eigen vector "face" of the data_buffer
  Eigen::Map<Vec> data_buffer_eigen(data_buffer_,winlen_);
  // least square fit the coeffecients
  Vec coeffs = A_.colPivHouseholderQr().solve(data_buffer_eigen);
  return coeffs;
}

int ScalarSavitzkyGolayFilter::GetOutput(real forward_param, int diff_order, real & result)
{
  if(n_added_ == 0)
    return -3;
  // make sure forward_param is in the allowed range
  forward_param = forward_param < 0.0 ? 0.0 : forward_param;
  forward_param = forward_param > 1.0 ? 1.0 : forward_param;

  // forward_param = 1 
  real query_val = 0.0 + real(winlen_)/2.0*forward_param;
  

  Vec coeffs = FitCoeffs();
  coeffs = diff_poly_coeffs(coeffs, diff_order);
  Vec query_vec(coeffs.size());
  for(int i=0;i<query_vec.size();i++)
    query_vec(i) = pow(query_val,i);
  result = coeffs.dot(query_vec);
  if(diff_order>0){
    result *= 1/pow(sample_time_, diff_order);
  }
  // if(diff_order % 2)
  //   result *= -1.0;
  if(IsInitialized())
    return 0;
  else
    return -1;
}

int ScalarSavitzkyGolayFilter::GetOutput(int diff_order, real & result)
{
  return GetOutput(0.0, diff_order, result);
}

bool ScalarSavitzkyGolayFilter::IsInitialized()
{
    if(n_added_ >= winlen_)
        return true;
    else
        return false;
}


SavitzkyGolayFilter::SavitzkyGolayFilter(int dim, int order, int winlen, real sample_time): dim_(dim){
  for(int i = 0; i<dim; i++){
    scalar_filters_.push_back(ScalarSavitzkyGolayFilter(order,winlen,sample_time));
  }
}

int SavitzkyGolayFilter::AddData(const Vec& inp){
  if(inp.size() == dim_){
    for(int i=0; i<dim_; i++)
      scalar_filters_[i].AddData(inp(i));
    return 0;
  }
  else
    return -2;
}

int SavitzkyGolayFilter::GetOutput(int diff_order, Vec & result){
  return GetOutput(0.0, diff_order, result);
}

int SavitzkyGolayFilter::GetOutput(real forward_param, int diff_order, Vec & result){
  result.resize(dim_);
  real tmp;
  for(int i=0; i<dim_; i++){
    scalar_filters_[i].GetOutput(forward_param, diff_order, tmp);
    result(i) = tmp;
  }
  if(IsInitialized())
    return 0;
  else
    return -1;
}

bool SavitzkyGolayFilter::IsInitialized(){
  return scalar_filters_[0].IsInitialized();
}


int SavitzkyGolayFilter::Filter(const Vec& input, int diff_order, Vec& output)
{
  int ret;
  ret = AddData(input);
  if (ret != 0)
    return ret;
  else
    return GetOutput(0, diff_order, output);
}
