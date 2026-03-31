//fft.h complex FFT function taken from Rulph's C31 book

void fft(COMPLEX *Y, int M, COMPLEX *w)       //input sample array, number of points
{
  COMPLEX temp1,temp2;            //temporary storage variables
  int i,j,k;                      //loop counter variables
  int upper_leg, lower_leg;       //index of upper/lower butterfly leg
  int leg_diff;                   //difference between upper/lower leg
  int num_stages=0;               //number of FFT stages, or iterations
  int index, step;                //index and step between twiddle factor
  i=1;                            //log(base 2) of # of points = # of stages
  do
  {
    num_stages+=1;
    i=i*2;
  } while (i!=M);

  leg_diff=M/2;                 //starting difference between upper & lower legs
  step=2;                     //step between values in twiddle.h              
  for (i=0;i<num_stages;i++)      //for M-point FFT                 
  {
    index=0;
    for (j=0;j<leg_diff;j++)
    {
      for (upper_leg=j;upper_leg<M;upper_leg+=(2*leg_diff))
      {
        lower_leg=upper_leg+leg_diff;
        temp1.real=(Y[upper_leg]).real + (Y[lower_leg]).real;
        temp1.imag=(Y[upper_leg]).imag + (Y[lower_leg]).imag;
        temp2.real=(Y[upper_leg]).real - (Y[lower_leg]).real;
        temp2.imag=(Y[upper_leg]).imag - (Y[lower_leg]).imag;
        (Y[lower_leg]).real=temp2.real*(w[index]).real-temp2.imag*(w[index]).imag;
        (Y[lower_leg]).imag=temp2.real*(w[index]).imag+temp2.imag*(w[index]).real;
        (Y[upper_leg]).real=temp1.real;
        (Y[upper_leg]).imag=temp1.imag;
      }
      index+=step;
    }
    leg_diff=leg_diff/2;
    step*=2;
  }
  j=0;
  for (i=1;i<(M-1);i++)           //bit reversal for resequencing data*/
  {
    k=M/2;
    while (k<=j)
    {
      j=j-k;
      k=k/2;
    }
    j=j+k;
    if (i<j)
    {
      temp1.real=(Y[j]).real;
      temp1.imag=(Y[j]).imag;
      (Y[j]).real=(Y[i]).real;
      (Y[j]).imag=(Y[i]).imag;
      (Y[i]).real=temp1.real;
      (Y[i]).imag=temp1.imag;
    }
  }
  return;
}                                 //end of fft()

     

