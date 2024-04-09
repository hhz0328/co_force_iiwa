#include "gtest/gtest.h"
#include <iostream>
#include "sg_filter.h"
#include <fstream>
#include <vector>
#include <string>
#include <sstream>


#define COMP_THRESHOLD 1e-6

using namespace SGF;

template<typename t>
void debug_info(t p){
  std::cout<<p<<std::endl;
}

template<typename T>
void load_matrix(const char *fname, T &target) {

  std::ifstream myfile(fname);
  EXPECT_TRUE(myfile);
  std::string line;
  float f;
  int n_rows=0;
  std::vector<float> v;
  while(getline(myfile,line)){
    ++n_rows;
    std::istringstream line_stream(line);
    while(!line_stream.eof()){
      line_stream>>f;
      v.push_back(f);
    }
  }
  int n_cols = v.size()/n_rows;
  target.resize(n_rows,n_cols);
  for(int i=0; i<n_rows;i++){
    for(int j=0;j<n_cols;j++){
      target(i,j) = v[i*n_cols + j];
    }
  }
}


template <typename R>
void assert_matrix_equal(const Eigen::Matrix<R,Eigen::Dynamic,Eigen::Dynamic>& m1, const Eigen::Matrix<R,Eigen::Dynamic,Eigen::Dynamic>& m2, double th){
  EXPECT_EQ(m1.rows(),m2.rows());
  EXPECT_EQ(m1.cols(),m2.cols());
  for (size_t row = 0; row < m1.rows(); ++row)
    {
      for (size_t col = 0; col < m1.cols(); ++col)
	{
	  EXPECT_NEAR(m1(row,col), m2(row,col),th);
	}
    }
}
template <typename T1,typename T2>
void assert_vector_equal(const T1 & m1, const T2& m2, double th){
  EXPECT_EQ(m1.rows(),m2.rows());
  EXPECT_EQ(m1.cols(),m2.cols());
  for (size_t row = 0; row < m1.rows(); ++row)
    {
      for (size_t col = 0; col < m1.cols(); ++col)
	{
	  EXPECT_NEAR(m1(row,col), m2(row,col),th);
	}
    }
}

TEST(CoefficientDifferentiatorTest, CoeffVectorLengthTest){
  Vec b(4);
  EXPECT_EQ(diff_poly_coeffs(b,1).size(),3);
  EXPECT_EQ(diff_poly_coeffs(b,2).size(),2);
}

TEST(CoefficientDifferentiatorTest, CorrectDifferentiationTest){
  Vec a(6);
  a << 1,1,1,1,1,1;
  Vec b(5);
  b << 1,2,3,4,5;
  Vec c(4);
  c << 2,6,12,20;
  assert_vector_equal(diff_poly_coeffs(a,1), b, COMP_THRESHOLD);
  assert_vector_equal(diff_poly_coeffs(a,2), c, COMP_THRESHOLD);
}

TEST(AddDataTest,test){
  int winlen = 3;
  ScalarSavitzkyGolayFilter filter(2,3,1);
  Vec test_add(9);
  test_add << 1,2,3,4,5,6,7,8,9;

  for(int i=0;i<test_add.size();i++){
    filter.AddData(test_add[i]);

    if(i<winlen-1){
      EXPECT_FALSE(filter.IsInitialized());
    }else{
      EXPECT_TRUE(filter.IsInitialized());
    }
    if(filter.IsInitialized()){
      assert_vector_equal(filter.GetData(),test_add.segment(i-winlen+1,winlen), COMP_THRESHOLD);
    }
  }
  
}

TEST(ConstructorTest, 5_51){
  ScalarSavitzkyGolayFilter filter(5,51,1);
  Mat A;
  load_matrix("5_51_matrix.txt",A);
  assert_matrix_equal(A, filter.GetPolynomialMatrix(),COMP_THRESHOLD);
}


TEST(CorrectnessTest, TestData1){
  int order = 3;
  int winlen = 11;
  float sample_time = 1e-3;
  ScalarSavitzkyGolayFilter filter(order,winlen,sample_time);
  Vec inp, outp, outp_d, outp_dd;

  load_matrix("test_data_1_inp.txt", inp);
  load_matrix("test_data_1_outp.txt", outp);
  load_matrix("test_data_1_outp_d.txt", outp_d);
  load_matrix("test_data_1_outp_dd.txt", outp_dd);

  Vec outp_gen(inp.size());
  Vec outp_d_gen(inp.size());
  Vec outp_dd_gen(inp.size());

  int test_size = inp.size();
  float tmp;
  for(int i=0;i<test_size;i++){
    filter.AddData(inp(i));
    filter.GetOutput(0, 0, tmp);
    outp_gen(i) = tmp;
    filter.GetOutput(0, 1, tmp);
    outp_d_gen(i) = tmp;
    filter.GetOutput(0, 2, tmp);
    outp_dd_gen(i) = tmp;

  }
  // dont care about testing when the filter is not initialized, i.e. skip the first winlen samples
  assert_vector_equal(outp.tail(test_size-winlen), outp_gen.tail(test_size-winlen), COMP_THRESHOLD);
  assert_vector_equal(outp_d.tail(test_size-winlen), outp_d_gen.tail(test_size-winlen), 0.02);
  //  debug_info(outp_d.tail(test_size-winlen) - outp_d_gen.tail(test_size-winlen));
  //  debug_info(outp_dd.tail(test_size-winlen) - outp_dd_gen.tail(test_size-winlen));

  // the accuracy is not as bad as it seems, about 0.5 % relative error here
  assert_vector_equal(outp_dd.tail(test_size-winlen), outp_dd_gen.tail(test_size-winlen), 0.5);
  
}
TEST(CorrectnessTest, TestData2){
  int order = 2;
  int winlen = 19;
  float sample_time = 1e-3;
  ScalarSavitzkyGolayFilter filter(order,winlen,sample_time);
  Vec inp, outp, outp_d, outp_dd;

  load_matrix("test_data_2_inp.txt", inp);
  load_matrix("test_data_2_outp.txt", outp);
  load_matrix("test_data_2_outp_d.txt", outp_d);
  load_matrix("test_data_2_outp_dd.txt", outp_dd);

  Vec outp_gen(inp.size());
  Vec outp_d_gen(inp.size());
  Vec outp_dd_gen(inp.size());

  int test_size = inp.size();
  float tmp;
  for(int i=0;i<test_size;i++){
    filter.AddData(inp(i));
    filter.GetOutput(0, 0, tmp);
    outp_gen(i) = tmp;
    filter.GetOutput(0, 1, tmp);
    outp_d_gen(i) = tmp;
    filter.GetOutput(0, 2, tmp);
    outp_dd_gen(i) = tmp;

  }

  // dont care about testing when the filter is not initialized, i.e. skip the first winlen samples
  assert_vector_equal(outp.tail(test_size-winlen), outp_gen.tail(test_size-winlen), 1e-4);
  assert_vector_equal(outp_d.tail(test_size-winlen), outp_d_gen.tail(test_size-winlen), 0.02);
  //  debug_info(outp_d.tail(test_size-winlen) - outp_d_gen.tail(test_size-winlen));
  //  debug_info(outp_dd.tail(test_size-winlen) - outp_dd_gen.tail(test_size-winlen));

  // the accuracy is not as bad as it seems, about 0.5 % relative error here
  assert_vector_equal(outp_dd.tail(test_size-winlen), outp_dd_gen.tail(test_size-winlen), 0.5);
  
}


TEST(MultiDimTes, DimensionTest){
  int order = 4;
  int winlen = 19;
  float sample_time = 1e-3;
  int dim = 2;
  SavitzkyGolayFilter filter(dim,order,winlen,1e-3);
  Vec inp(dim);
  Vec outp;
  filter.Filter(inp,0,outp);
  EXPECT_EQ(outp.size(), dim);
}

TEST(MultiDimTes, AddDataReturnCodes){
  int order = 4;
  int winlen = 19;
  float sample_time = 1e-3;
  int dim = 2;
  SavitzkyGolayFilter filter(dim,order,winlen,1e-3);
  Vec inp(dim+1);
  EXPECT_EQ(filter.AddData(inp), -2);
  inp.resize(dim);
  EXPECT_EQ(filter.AddData(inp), 0);
}

TEST(MultiDimTest, CorrectnessTest){
  int order = 4;
  int winlen = 19;
  float sample_time = 1e-3;
  int dim = 2;
  SavitzkyGolayFilter filter(dim,order,winlen,1e-3);
  Vec inp1, outp1, outp_d1, outp_dd1;
  Vec inp2, outp2, outp_d2, outp_dd2;


  load_matrix("test_data_2_inp.txt", inp1);
  int test_size = inp1.size();
  load_matrix("test_data_2_outp.txt", outp1);
  load_matrix("test_data_2_outp_d.txt", outp_d1);
  load_matrix("test_data_2_outp_dd.txt", outp_dd1);
  load_matrix("test_data_3_inp.txt", inp2);
  load_matrix("test_data_3_outp.txt", outp2);
  load_matrix("test_data_3_outp_d.txt", outp_d2);
  load_matrix("test_data_3_outp_dd.txt", outp_dd2);

  Vec outp_gen1(test_size);
  Vec outp_d_gen1(test_size);
  Vec outp_dd_gen1(test_size);
  Vec outp_gen2(test_size);
  Vec outp_d_gen2(test_size);
  Vec outp_dd_gen2(test_size);

  Vec tmp(dim);
  for(int i=0; i<test_size; i++){
    tmp << inp1(i), inp2(i);
    filter.AddData(tmp);
    filter.GetOutput(0, tmp);
    outp_gen1(i) = tmp(0);
    outp_gen2(i) = tmp(1);
    filter.GetOutput(1, tmp);
    outp_d_gen1(i) = tmp(0);
    outp_d_gen2(i) = tmp(1);
    filter.GetOutput(2, tmp);
    outp_dd_gen1(i) = tmp(0);
    outp_dd_gen2(i) = tmp(1);
  }
  
  assert_vector_equal(outp1.tail(test_size-winlen), outp_gen1.tail(test_size-winlen), 1e-4);
  assert_vector_equal(outp_d1.tail(test_size-winlen), outp_d_gen1.tail(test_size-winlen), 0.02);
  assert_vector_equal(outp_dd1.tail(test_size-winlen), outp_dd_gen1.tail(test_size-winlen), 0.5);

  assert_vector_equal(outp2.tail(test_size-winlen), outp_gen2.tail(test_size-winlen), 1e-4);
  assert_vector_equal(outp_d2.tail(test_size-winlen), outp_d_gen2.tail(test_size-winlen), 0.04);
  assert_vector_equal(outp_dd2.tail(test_size-winlen), outp_dd_gen2.tail(test_size-winlen), 0.5);

}




int main(int argc, char *argv[]){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
